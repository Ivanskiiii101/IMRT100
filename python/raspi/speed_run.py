# speed_run.py - IMRT100 maze solver, robot-vacuum style.
#
# No wall-following, no corridor-centring, no continuous steering math -
# just drive straight until something is close, then react:
#   - front close -> stop, back up, turn toward whichever side has more
#     room, keep driving
#   - either side close -> stop, back up, nudge away, keep driving
# A junction, a dead end, and a plain wall dead ahead all just look like
# "front is close" here, so there's no separate junction-vs-dead-end
# decision to get wrong - one reaction covers all of them. Drift toward a
# wall on a straight stretch gets corrected the same way any other
# obstacle does: it's just a "side got close" event.
#
# Hardware, confirmed by direct testing on this robot:
#   dist_1 = right sensor, dist_2 = left, dist_3 = front, dist_4 = rear
#   (rear is read but never used for a decision)
#   motor_1 = left wheel, motor_2 = right wheel, positive = forward
#
# Two things kept from testing the wall-following versions of this file,
# because dropping them caused real, repeatable failures on this robot:
#   - a close/blocked reading needs a couple of confirmed consecutive
#     ticks, not one - these ultrasonic sensors are noisy enough that a
#     single reading was enough to trigger a false bounce
#   - a turn only ends once the front reads clear for several consecutive
#     ticks, not one - ending it on a single lucky reading stops the turn
#     before the robot has actually turned clear, so the very next tick
#     immediately re-triggers another turn; a few of those in a row look
#     like the robot spinning in place and ending up back where it started
# Everything else is intentionally as simple as it can be - no threads,
# no zones, no smooth speed ramp. One speed, a couple of thresholds.
#
# The sound is a fire-and-forget subprocess, not a background thread:
# Popen starts the player and returns immediately, so it never blocks the
# control loop, and there's nothing to explicitly turn off - the player
# process just exits on its own once the clip finishes.

from collections import deque
from pathlib import Path
import statistics
import subprocess
import sys
import time

import imrt_robot_serial


SENSOR_RIGHT = 1
SENSOR_LEFT = 2
SENSOR_FRONT = 3
SENSOR_REAR = 4  # read every tick, never used for a decision

MOTOR_LEFT_SIGN = 1
MOTOR_RIGHT_SIGN = 1

CRUISE_SPEED = 210      # was 230, dialed back down - 190 was the last
                        # confirmed-good speed, 230 kept U-turning even
                        # with the bigger stopping margins below, so
                        # 210 splits the difference to narrow down where
                        # the actual working ceiling is. CONTROL_PERIOD
                        # and the STOP_CM margins are left at their 230
                        # values on purpose - loosening them back up for
                        # a lower speed would only remove margin for no
                        # benefit; leaving them gives this test more
                        # safety buffer than 210 strictly needs, not less.
# Raised from 140. Every turn commands both wheels in opposite directions
# at this speed (one forward, one backward) - if one motor doesn't have
# enough command strength to actually overcome friction in reverse at a
# given magnitude, the robot arcs around the stuck wheel instead of
# pivoting on the spot, which throws off rotate_until_clear() (it assumes
# an in-place pivot when deciding how long "clear" should take). Raising
# this pushes both motors further past that point. A jump to 150 was
# tried once before and made things worse, but that was diagnosed as
# overshoot from TURN_STEP_SECONDS still being 0.05 at the time - that's
# since been tightened to 0.04, so this is a genuinely different test,
# not a repeat of the same failed change.
TURN_SPEED = 170
BACKUP_SPEED = 120

# This is also where bounce_off_front() decides which way to turn (reads
# left/right right after triggering), so it doubles as "how close before
# comparing sides at a junction" - too far out and a small opening hasn't
# actually come into view yet, so the direction choice is a guess. Pulled
# back down from 50 toward the original 35 for that reason, but not all
# the way: 50 was sized for CRUISE_SPEED's momentum (now 210, still
# faster than the 150 that 35 was originally set for), so this trades
# some of that stopping margin back for a closer, more accurate look
# before turning. Worth watching for more front contact than before.
FRONT_STOP_CM = 40
# Back down from 16 to 12: that was sized for CRUISE_SPEED=230, which got
# dialed back to 210, so it was more cautious than the current speed
# needs - and, separately, it had no debounce at all (unlike front, which
# needs 2 confirmed ticks), so a single close reading while squeezing
# through an ordinary narrow stretch triggered a full stop-and-nudge on
# its own. Both together meant a merely-tight-but-passable corridor could
# derail otherwise fine forward progress even with the front wide open -
# SIDE_BLOCK_CONFIRM_SAMPLES below fixes the debounce half of that.
SIDE_STOP_CM = 12
NO_ECHO_RECOVERY_CM = 80  # see the 255-sentinel handling in read_distances()

FRONT_BLOCK_CONFIRM_SAMPLES = 2
FRONT_CLEAR_CONFIRM_SAMPLES = 3  # symmetric with the above - see docstring
SIDE_BLOCK_CONFIRM_SAMPLES = 2   # same idea, applied to the side check

BACKUP_SECONDS = 0.2
STUCK_BACKUP_SECONDS = 0.6   # a bigger backup once bouncing repeatedly
STUCK_THRESHOLD = 2          # with no forward progress in between
MAX_CONSECUTIVE_STUCK = 6    # give up rather than bounce forever

SIDE_AVOID_BACKUP_SECONDS = 0.10
# This turn is fixed-duration, not condition-checked like the front turn -
# so when TURN_SPEED went 140->170 (to fix the wheel dead-zone issue),
# this started sweeping a proportionally bigger angle in the same time
# without anyone noticing, which is what was causing the repeated
# left/right bounce-ping-pong down a tight corridor: an over-rotating
# nudge away from one wall swings far enough to trigger the other side
# immediately. Scaled back down to hold the actual angle turned roughly
# constant: 0.15*140/170=0.124, rounded to 0.12.
SIDE_AVOID_TURN_SECONDS = 0.12

# Checked more often, not faster: raising TURN_SPEED instead of tightening
# this was tried and made things worse - the robot swept a bigger angle
# between checks and overshot past "just clear" into the next thing, which
# then needed extra bounce cycles to recover from. Same idea for
# CONTROL_PERIOD below - the sample *counts* in FRONT_BLOCK_CONFIRM_SAMPLES
# etc are untouched, so the noise-rejection they give is unchanged; the
# same number of confirm ticks just takes less real time to resolve.
TURN_STEP_SECONDS = 0.04
MAX_TURN_SECONDS = 1.7  # safety cap only - see rotate_until_clear()

EXIT_OPEN_CM = 180
EXIT_CONFIRM_SAMPLES = 12
START_GRACE_SECONDS = 5.0  # a spacious start bay can look like the exit

SENSOR_NO_ECHO_RAW = 250
# Tightened again alongside CRUISE_SPEED, same approach that fixed the
# loop/U-turn last round: 190*0.04=7.6 (speed*period); holding that
# constant at CRUISE_SPEED=230 gives 7.6/230=0.033, rounded down to 0.03 -
# same distance covered during the confirm window as at 190, not more.
CONTROL_PERIOD = 0.03

# Real speaker output (not the piezo GPIO buzzer) - played via an external
# player process, not GPIO, so it can handle an actual MP3 file.
AUDIO_FILE = Path(__file__).with_name("meow-meow-meow-tiktok.mp3")
# Raised from 40 to stay outside the new FRONT_STOP_CM=50 - otherwise the
# robot bounces away before ever getting close enough to reach this.
SOUND_TRIGGER_CM = 60  # play once when front gets this close


def clamp(value, lower=-500, upper=500):
    return max(lower, min(upper, int(value)))


def play_sound():
    # Fire-and-forget - see the module docstring for why this is safe to
    # call from inside the control loop without a thread. mpg123 is the
    # Linux equivalent of macOS's afplay; -q keeps it from printing track
    # info to stdout, which would otherwise clutter the status line above.
    try:
        subprocess.Popen(["mpg123", "-q", str(AUDIO_FILE)])
    except Exception as error:
        print(f"(sound unavailable: {error})")


class MazeSolver:
    def __init__(self, robot):
        self.robot = robot
        self.history = {number: deque(maxlen=5) for number in (1, 2, 3, 4)}
        self.exit_open_count = 0
        self.front_blocked_streak = 0
        self.side_blocked_streak = 0
        # Consecutive bounces with no successful forward driving in
        # between - not reset until a normal driving tick happens.
        self.consecutive_blocked = 0
        # True while front is still inside SOUND_TRIGGER_CM, so the sound
        # fires once on the approach, not once per tick for as long as
        # it's close - reset the moment front is clear again.
        self.sound_triggered = False

    def send(self, left, right):
        self.robot.send_command(
            clamp(left * MOTOR_LEFT_SIGN), clamp(right * MOTOR_RIGHT_SIGN)
        )

    def stop(self):
        for _ in range(3):
            self.send(0, 0)
            time.sleep(0.05)

    def timed_drive(self, left, right, duration):
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time and not self.robot.shutdown_now:
            self.send(left, right)
            time.sleep(0.05)

    def rotate_until_clear(self, left, right):
        deadline = time.monotonic() + MAX_TURN_SECONDS
        clear_streak = 0
        while time.monotonic() < deadline and not self.robot.shutdown_now:
            self.send(left, right)
            time.sleep(TURN_STEP_SECONDS)
            front = self.read_distances()[SENSOR_FRONT]
            clear_streak = clear_streak + 1 if front >= FRONT_STOP_CM else 0
            if clear_streak >= FRONT_CLEAR_CONFIRM_SAMPLES:
                break
        self.stop()

    def bounce_off_front(self):
        self.stop()
        backup = (
            STUCK_BACKUP_SECONDS
            if self.consecutive_blocked >= STUCK_THRESHOLD
            else BACKUP_SECONDS
        )
        self.timed_drive(-BACKUP_SPEED, -BACKUP_SPEED, backup)

        distances = self.read_distances()
        left = distances[SENSOR_LEFT]
        right = distances[SENSOR_RIGHT]
        if right >= left:
            self.rotate_until_clear(TURN_SPEED, -TURN_SPEED)
        else:
            self.rotate_until_clear(-TURN_SPEED, TURN_SPEED)

    def bounce_off_side(self, close_sensor):
        self.stop()
        self.timed_drive(-BACKUP_SPEED, -BACKUP_SPEED, SIDE_AVOID_BACKUP_SECONDS)
        if close_sensor == SENSOR_LEFT:
            self.timed_drive(TURN_SPEED, -TURN_SPEED, SIDE_AVOID_TURN_SECONDS)
        else:
            self.timed_drive(-TURN_SPEED, TURN_SPEED, SIDE_AVOID_TURN_SECONDS)
        self.stop()

    def read_distances(self):
        raw = {
            1: self.robot.get_dist_1(),
            2: self.robot.get_dist_2(),
            3: self.robot.get_dist_3(),
            4: self.robot.get_dist_4(),
        }
        for number, value in raw.items():
            # The sensor reports 255 both for "nothing in range" and for an
            # object closer than its minimum range. If the last reading was
            # already close, treat a jump to 255 as still blocked, not open.
            if value >= SENSOR_NO_ECHO_RAW and self.history[number]:
                previous = statistics.median(self.history[number])
                if previous < NO_ECHO_RECOVERY_CM:
                    value = 0
            self.history[number].append(value)
        return {
            number: statistics.median(values)
            for number, values in self.history.items()
        }

    def run(self):
        print("speed_run: bump-and-turn maze solver running. Ctrl+C to stop.")
        started_at = time.monotonic()

        while not self.robot.shutdown_now:
            tick_started = time.monotonic()
            distances = self.read_distances()
            left = distances[SENSOR_LEFT]
            front = distances[SENSOR_FRONT]
            right = distances[SENSOR_RIGHT]

            print(
                f"left={left:5.1f}  front={front:5.1f}  right={right:5.1f}",
                end="\r",
                flush=True,
            )

            if front <= SOUND_TRIGGER_CM:
                if not self.sound_triggered:
                    play_sound()
                    self.sound_triggered = True
            else:
                self.sound_triggered = False

            if min(left, front, right) >= EXIT_OPEN_CM:
                self.exit_open_count += 1
            else:
                self.exit_open_count = 0

            past_start_grace = time.monotonic() - started_at >= START_GRACE_SECONDS
            if past_start_grace and self.exit_open_count >= EXIT_CONFIRM_SAMPLES:
                self.stop()
                print("\n>>> Exit found - wide open on all sides. Stopping.")
                return

            if front <= FRONT_STOP_CM:
                self.front_blocked_streak += 1
            else:
                self.front_blocked_streak = 0

            if self.front_blocked_streak >= FRONT_BLOCK_CONFIRM_SAMPLES:
                self.front_blocked_streak = 0
                self.consecutive_blocked += 1
                if self.consecutive_blocked > MAX_CONSECUTIVE_STUCK:
                    self.stop()
                    print(f"\n>>> Stopping: stuck after {MAX_CONSECUTIVE_STUCK} "
                          "bounces with no progress.")
                    return
                print(f"\n>>> BOUNCE front ({self.consecutive_blocked}/"
                      f"{MAX_CONSECUTIVE_STUCK}) left={left:.0f} right={right:.0f}")
                self.bounce_off_front()
                self.side_blocked_streak = 0
                continue

            if left < SIDE_STOP_CM or right < SIDE_STOP_CM:
                self.side_blocked_streak += 1
            else:
                self.side_blocked_streak = 0

            if self.side_blocked_streak >= SIDE_BLOCK_CONFIRM_SAMPLES:
                self.side_blocked_streak = 0
                close_sensor = SENSOR_LEFT if left < right else SENSOR_RIGHT
                side = "LEFT" if close_sensor == SENSOR_LEFT else "RIGHT"
                print(f"\n>>> BOUNCE {side} at left={left:.0f} right={right:.0f}")
                self.bounce_off_side(close_sensor)
                self.front_blocked_streak = 0
                continue

            self.consecutive_blocked = 0
            self.send(CRUISE_SPEED, CRUISE_SPEED)

            remaining = CONTROL_PERIOD - (time.monotonic() - tick_started)
            if remaining > 0:
                time.sleep(remaining)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    robot = imrt_robot_serial.IMRTRobotSerial()

    try:
        robot.connect(port)
        robot.run()
        MazeSolver(robot).run()
    except Exception as error:
        print(f"\nRobot program stopped because of an error: {error}")
        raise
    finally:
        if hasattr(robot, "serial_port_"):
            try:
                MazeSolver(robot).stop()
            except Exception:
                pass
        try:
            robot._shutdown()
        except Exception:
            pass
        print("Robot stopped.")


if __name__ == "__main__":
    main()
