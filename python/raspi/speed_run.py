# speed_run.py - IMRT100 maze solver, robot-vacuum style.
#
# No wall-following, no corridor-centring - just drive straight until the
# front is close, then react:
#   - front close -> stop, back up, turn toward whichever side has more
#     room, keep driving
#   - either side close -> a short, bounded steer away from it (both
#     wheels stay forward, one just slower for a moment - a nudge, not a
#     pivot), then straight back to normal driving and a fresh read next
#     tick. No stop, no backing up. This replaced an earlier version that
#     leaned continuously for as long as the reading stayed close -
#     that only slowed the approach rather than committing to a real
#     heading change, which wasn't always enough to avoid contact, and
#     could accumulate more heading drift than a robot that recovers to
#     straight and re-reads after each small, bounded correction.
# A junction, a dead end, and a plain wall dead ahead all just look like
# "front is close" here, so there's no separate junction-vs-dead-end
# decision to get wrong - one reaction covers all of them.
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

CRUISE_SPEED = 191      # was 182, +5% - CONTROL_PERIOD below is
                        # tightened to match, same approach used
                        # throughout this file's tuning history.
TURN_SPEED = 140
BACKUP_SPEED = 120
# The inner wheel's speed during the side-nudge - well below CRUISE_SPEED
# so the heading change is sharp enough to matter in a short burst, but
# still a real forward speed, never a stop or a reverse. What actually
# matters for the nudge's turn rate is the *gap* to CRUISE_SPEED, not
# this value on its own - held at a gap of 95 again (191-95=96), so the
# ~5 degree calibration on SIDE_NUDGE_SECONDS still holds instead of
# quietly turning a bit sharper each time CRUISE_SPEED goes up.
SIDE_ADJUST_SPEED = 96
# How long the nudge lasts - both wheels stay forward (unlike a pivot,
# where one reverses), so this needs to be long enough for the speed gap
# to actually turn the heading a meaningful amount, roughly ~5 degrees.
# Not calibrated against real angle measurements - halved from 0.15
# (roughly ~10 degrees) on the assumption that duration scales close to
# linearly with angle for a turn this small. Tune further if it's still
# off on the robot.
SIDE_NUDGE_SECONDS = 0.075

FRONT_STOP_CM = 35
# Below this on either side, that side gets one side_nudge() - a single
# bounded correction, not a sustained lean. No confirm-samples debounce
# here on purpose: a wrong reaction is just one short nudge with the
# robot still moving forward the whole time, not a committed stop-and-
# turn, so a single noisy reading costs very little.
# Raised from 12 - CRUISE_SPEED has grown to 165 since this was set, and
# more speed means more distance covered before a correction has time to
# create real clearance. Triggering earlier gives it more room to work
# with before actual contact.
SIDE_ADJUST_CM = 18
NO_ECHO_RECOVERY_CM = 80  # see the 255-sentinel handling in read_distances()

FRONT_BLOCK_CONFIRM_SAMPLES = 2
FRONT_CLEAR_CONFIRM_SAMPLES = 3  # symmetric with the above - see docstring

BACKUP_SECONDS = 0.2
STUCK_BACKUP_SECONDS = 0.6   # a bigger backup once bouncing repeatedly
STUCK_THRESHOLD = 2          # with no forward progress in between
MAX_CONSECUTIVE_STUCK = 6    # give up rather than bounce forever

# After backing up, bounce_off_front() compares left/right to pick a
# turn direction - but if one side has been consistently close for a
# while right up until the bounce (e.g. hugging a wall down a long
# corridor that then opens up), the median-of-5 history for that sensor
# is still mostly "close" samples even once the robot is sitting right at
# the opening. One reading isn't enough to move a median off stale data -
# take a few fresh ones first so the comparison reflects what's actually
# there now, not what was there several ticks ago.
DIRECTION_SETTLE_READS = 3
DIRECTION_SETTLE_SECONDS = 0.05

# If a bounce fires again within this long of the last turn, treat it as
# still resolving the same junction (e.g. drove into a dead end, hit its
# far wall, and is now backing out) and go the opposite way from last
# time, instead of re-deriving the same answer from readings that
# haven't meaningfully changed. Long enough to comfortably cover driving
# into a shallow dead-end pocket and bouncing back off its far wall;
# short enough that hitting an unrelated wall much later in a genuinely
# different part of the maze still gets a fresh decision.
RECENT_TURN_SECONDS = 3.0

TURN_STEP_SECONDS = 0.05
MAX_TURN_SECONDS = 1.7  # safety cap only - see rotate_until_clear()

EXIT_OPEN_CM = 180
EXIT_CONFIRM_SAMPLES = 12
START_GRACE_SECONDS = 5.0  # a spacious start bay can look like the exit

SENSOR_NO_ECHO_RAW = 250
# Tightened alongside CRUISE_SPEED again: 182*0.044=8.008 (speed*period);
# holding that constant at CRUISE_SPEED=191 gives 8.008/191=0.0419,
# rounded down to 0.041 - same distance covered during the confirm
# window as before, not more.
CONTROL_PERIOD = 0.041

# Real speaker output (not the piezo GPIO buzzer) - played via an external
# player process, not GPIO, so it can handle an actual MP3 file.
AUDIO_FILE = Path(__file__).with_name("meow-meow-meow-tiktok.mp3")
SOUND_TRIGGER_CM = 40  # play once when front gets this close


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
        # Consecutive bounces with no successful forward driving in
        # between - not reset until a normal driving tick happens. Still
        # used for the backup-further/give-up escalation - see
        # STUCK_THRESHOLD/MAX_CONSECUTIVE_STUCK.
        self.consecutive_blocked = 0
        # Which way the most recent turn went (True=right, False=left),
        # and when - see bounce_off_front(). Deliberately time-based, not
        # tied to consecutive_blocked: driving even briefly into a dead
        # end before hitting its far wall counts as "forward progress"
        # and resets consecutive_blocked to 0, so a zero-progress-only
        # check never catches "I just tried this branch and it was a dead
        # end" - only "I'm stuck in literally the same spot."
        self.last_turn_was_right = None
        self.last_turn_at = None
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
        # Decide direction first, from right here at the wall - not after
        # backing away from it. A turn is often a fairly tight corner, not
        # a wide room, so backing up before looking can retreat the true
        # opening out of view, leaving two similar-looking corridor walls
        # to compare instead of the actual junction.
        self.stop()

        recently_turned = (
            self.last_turn_was_right is not None
            and self.last_turn_at is not None
            and time.monotonic() - self.last_turn_at < RECENT_TURN_SECONDS
        )
        if recently_turned:
            # Turned this recently, and we're already bouncing again -
            # very likely still resolving the same junction (backed into
            # a dead end and out again, or stuck in place). Re-comparing
            # left/right tends to land on the same answer, since the
            # readings haven't meaningfully changed either way. Go the
            # other way instead of asking the same question again.
            turn_right = not self.last_turn_was_right
        else:
            for _ in range(DIRECTION_SETTLE_READS):
                distances = self.read_distances()
                time.sleep(DIRECTION_SETTLE_SECONDS)
            left = distances[SENSOR_LEFT]
            right = distances[SENSOR_RIGHT]
            turn_right = right >= left

        # Whatever decided the direction above, never turn toward a side
        # that's close right now - checked unconditionally, every time,
        # so it can't be silently skipped by the recently_turned branch
        # the way a check nested only in the other branch could be.
        distances = self.read_distances()
        left = distances[SENSOR_LEFT]
        right = distances[SENSOR_RIGHT]
        if turn_right and right < SIDE_ADJUST_CM:
            turn_right = False
        elif not turn_right and left < SIDE_ADJUST_CM:
            turn_right = True

        self.last_turn_was_right = turn_right
        self.last_turn_at = time.monotonic()

        # Now back up - purely for physical clearance to pivot without
        # catching a corner, using the direction already decided above.
        backup = (
            STUCK_BACKUP_SECONDS
            if self.consecutive_blocked >= STUCK_THRESHOLD
            else BACKUP_SECONDS
        )
        self.timed_drive(-BACKUP_SPEED, -BACKUP_SPEED, backup)
        self.stop()  # timed_drive() doesn't stop on its own when it ends
        if turn_right:
            self.rotate_until_clear(TURN_SPEED, -TURN_SPEED)
        else:
            self.rotate_until_clear(-TURN_SPEED, TURN_SPEED)

    def side_nudge(self, close_sensor):
        # Both wheels stay forward - this is a steer, not a pivot. Fixed,
        # short duration: after it ends, run() reads fresh distances on
        # its very next iteration, rather than continuing to react to a
        # reading that's now out of date.
        if close_sensor == SENSOR_RIGHT:
            self.timed_drive(SIDE_ADJUST_SPEED, CRUISE_SPEED, SIDE_NUDGE_SECONDS)
        else:
            self.timed_drive(CRUISE_SPEED, SIDE_ADJUST_SPEED, SIDE_NUDGE_SECONDS)

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
        print("speed_run: maze solver running. Ctrl+C to stop.")
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
                continue

            self.consecutive_blocked = 0

            if right < SIDE_ADJUST_CM:
                self.side_nudge(SENSOR_RIGHT)
                continue
            if left < SIDE_ADJUST_CM:
                self.side_nudge(SENSOR_LEFT)
                continue

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

