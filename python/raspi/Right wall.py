# Right-hand wall follower for IMRT100.
#
# Clean rebuild combining everything confirmed across testing this project.
# Core decision, per corridor tick, is simple by design:
#
#   right sees a wall nearby  -> MOVE_FORWARD (front permitting)
#   right sees no wall nearby -> TURN_RIGHT (a real opening)
#   front blocked             -> TURN_LEFT (dead end)
#
# Turn detection and hug-distance steering are deliberately kept as two
# separate concerns with two separate thresholds. Earlier versions reused
# one right-side number for both jobs and broke the same way each time -
# normal hug-distance variation looked identical to a real opening, so it
# kept trying to turn in the middle of an ordinary corridor. RIGHT_OPEN_CM
# only decides "is the wall genuinely gone" and is set from an actual test
# log where the robot read 26-90cm during ordinary (if erratic, undriven-
# straight) driving - a real opening should read far higher than that.
# RIGHT_TARGET_CM/LEFT_MIN_CM are unrelated: a light, always-on steering
# correction to counteract the two motors' mismatch (proven repeatedly
# earlier in this project) drifting the heading unopposed. Without it the
# robot drives dead straight in whatever direction the mismatch points it.
#
# Everything else here is infrastructure proven necessary by repeated
# testing, not guesses:
#   - sensor mapping confirmed by hand: dist_1=right, dist_2=left,
#     dist_3=centre, dist_4=behind. The rear sensor is intentionally not
#     used for any decision.
#   - motor_1/motor_2 -> left/right wheel, positive = forward, confirmed
#     directly on the robot with motor_direction_test.py
#   - raw 255 readings right after a close reading are treated as still
#     blocked, not as open space (ultrasonic sensors report 255 both for
#     "nothing in range" and for an object closer than their minimum range)
#   - readings are smoothed over the last 5 samples - sensors mounted close
#     together can pick up a neighbour's echo for a tick or two
#   - a turn/opening only counts once it holds for several consecutive
#     ticks, not one reading - a single noisy sample was directly
#     responsible for false turns and full spins in earlier testing
#   - forward speed eases off approaching the front threshold instead of
#     holding full speed to the last centimetre
#   - a side wall closer than SIDE_STOP_CM gets an immediate stop-and-move-
#     away response, since gentle driving-forward correction can't always
#     avoid a wall that's already that close
#   - a turn rotates in small increments, checking the front after each
#     one, instead of committing to a fixed duration for a specific angle

from collections import deque
import statistics
import sys
import time

import imrt_robot_serial


SENSOR_RIGHT = 1
SENSOR_LEFT = 2
SENSOR_CENTRE = 3
SENSOR_BEHIND = 4  # read by the protocol, never used for a decision here

# Motor commands. The Arduino accepts -500 to +500.
FORWARD_SPEED = 135
MIN_FORWARD_SPEED = 90  # floor for the slowdown ramp - below this the
                         # motors likely can't overcome friction
TURN_SPEED = 140
BACKUP_SPEED = 120

# Front: distance in centimetres below which "something is directly ahead."
FRONT_STOP_CM = 30
# Start easing forward speed down from this far out, so it's already slow
# by the time FRONT_STOP_CM is reached instead of braking abruptly.
FRONT_SLOWDOWN_CM = 80

# Right: a reading at or above this means the wall is genuinely gone. Set
# well above the 26-90cm range seen during normal (if erratic) driving in
# testing, and well below what an actual opening reads.
RIGHT_OPEN_CM = 150
# A junction can also show up as a sudden rise from wherever right was
# recently tracking, even if it doesn't reach RIGHT_OPEN_CM outright (e.g.
# the corridor beyond the junction isn't very wide). A rise at least this
# big from the recent minimum counts too - either signal is enough,
# subject to the same confirm-samples debounce as before.
RIGHT_JUMP_CM = 60
# After turning at a junction, a reading at or below this counts as having
# found the wall again - well below RIGHT_OPEN_CM, comfortably above
# ordinary driving noise around RIGHT_TARGET_CM.
RIGHT_REACQUIRE_CM = 90
# How long each turn-then-search attempt gets to find the wall before
# giving up and turning again, and how many attempts before giving up
# entirely at one junction.
JUNCTION_SEARCH_SECONDS = 1.2
MAX_JUNCTION_ATTEMPTS = 3

# The two motors aren't perfectly matched - proven repeatedly earlier in
# this project - so driving with literally equal motor speeds lets the
# robot drift steadily toward one wall with nothing to counteract it.
# MOVE_FORWARD steers continuously to hold RIGHT_TARGET_CM from the right
# wall (a true single target, not a dead-zone band, so it's always actively
# correcting except at the exact setpoint) and, independently, pushes away
# if it ever gets closer than LEFT_MIN_CM to the left wall - a floor, not a
# target, so it doesn't fight to hold an exact left distance too. Both are
# kept well clear of RIGHT_OPEN_CM so neither can be confused with a real
# opening.
RIGHT_TARGET_CM = 50
LEFT_MIN_CM = 70
STEER_GAIN = 2
MAX_STEER_CORRECTION = 35

# A side wall this close needs a real stop-and-move-away response, not just
# waiting for the next decision tick.
SIDE_STOP_CM = 10

EXIT_OPEN_CM = 180
EXIT_CONFIRM_SAMPLES = 12   # 1.2 seconds of open space
# A spacious starting bay can read just as open as the real finish area on
# every sensor - ignore exit detection for this long after starting.
START_GRACE_SECONDS = 5.0

# Require the trigger to hold for this many consecutive control-loop ticks
# before actually committing to a turn.
RIGHT_OPEN_CONFIRM_SAMPLES = 3
FRONT_BLOCK_CONFIRM_SAMPLES = 2

# Raw sensor value (0-255) meaning "no echo received." Also happens when an
# object is closer than the sensor's minimum range - see read_distances().
SENSOR_NO_ECHO_RAW = 250

CONTROL_PERIOD = 0.08       # 12.5 Hz; safely inside Arduino's 500 ms timeout
TURN_STEP_SECONDS = 0.05
MAX_TURN_SECONDS = 1.7      # safety cap per turn (covers roughly 180 degrees)
SIDE_AVOID_BACKUP_SECONDS = 0.10
SIDE_AVOID_TURN_SECONDS = 0.15

# Change either sign if a positive command drives that motor backwards.
# Confirmed correct (motor_1=left, motor_2=right, positive=forward) with
# motor_direction_test.py.
MOTOR_1_SIGN = 1
MOTOR_2_SIGN = 1


def clamp(value, lower=-500, upper=500):
    return max(lower, min(upper, int(value)))


class RightWallFollower:
    def __init__(self, robot):
        self.robot = robot
        # maxlen=5: with sensors mounted close together, one can occasionally
        # pick up a neighbour's echo (crosstalk) and report a bogus close
        # reading for a tick or two. A wider median window needs more than
        # one or two bad-in-a-row samples to actually move the result.
        self.history = {number: deque(maxlen=5) for number in (1, 2, 3, 4)}
        self.exit_open_count = 0
        self.right_open_streak = 0
        self.front_blocked_streak = 0
        # Recent smoothed right readings, used only to detect a sudden rise
        # (a junction) - separate from self.history, which is for cleaning
        # up raw sensor noise.
        self.recent_right = deque(maxlen=5)

    def send(self, motor_1, motor_2):
        self.robot.send_command(
            clamp(motor_1 * MOTOR_1_SIGN),
            clamp(motor_2 * MOTOR_2_SIGN),
        )

    def stop(self):
        # Repeat stop commands so one damaged serial packet cannot leave an old
        # command active. The Arduino also has its own 500 ms timeout.
        for _ in range(3):
            self.send(0, 0)
            time.sleep(0.05)

    def timed_drive(self, motor_1, motor_2, duration):
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time and not self.robot.shutdown_now:
            self.send(motor_1, motor_2)
            time.sleep(0.05)

    def rotate_until_clear(self, motor_1, motor_2):
        deadline = time.monotonic() + MAX_TURN_SECONDS
        while time.monotonic() < deadline and not self.robot.shutdown_now:
            self.send(motor_1, motor_2)
            time.sleep(TURN_STEP_SECONDS)
            centre = self.read_distances()[SENSOR_CENTRE]
            if centre >= FRONT_STOP_CM:
                break
        self.stop()

    def turn_right(self):
        self.stop()
        self.rotate_until_clear(TURN_SPEED, -TURN_SPEED)

    def turn_left(self):
        self.stop()
        self.rotate_until_clear(-TURN_SPEED, TURN_SPEED)

    def advance_and_search(self, max_duration):
        # Drive forward using the same steering correction as normal
        # driving - while right still reads far this naturally curves
        # toward finding a wall, rather than driving dead straight through
        # the junction. Returns "found", "front_blocked", or "timeout".
        deadline = time.monotonic() + max_duration
        while time.monotonic() < deadline and not self.robot.shutdown_now:
            distances = self.read_distances()
            centre = distances[SENSOR_CENTRE]
            right = distances[SENSOR_RIGHT]
            left = distances[SENSOR_LEFT]
            if centre <= FRONT_STOP_CM:
                self.stop()
                return "front_blocked"
            if right <= RIGHT_REACQUIRE_CM:
                self.stop()
                return "found"
            right_error = right - RIGHT_TARGET_CM
            left_error = max(0.0, LEFT_MIN_CM - left)
            correction = clamp(
                (right_error + left_error) * STEER_GAIN,
                -MAX_STEER_CORRECTION, MAX_STEER_CORRECTION,
            )
            self.send(MIN_FORWARD_SPEED + correction,
                     MIN_FORWARD_SPEED - correction)
            time.sleep(0.05)
        self.stop()
        return "timeout"

    def navigate_junction(self):
        # A junction can be wider than one pivot covers - turn, advance a
        # bit looking for the wall, and if it's still not there, advance
        # further (stopping short of the front wall) and turn again, rather
        # than committing to a single turn and hoping.
        for attempt in range(1, MAX_JUNCTION_ATTEMPTS + 1):
            print(f"    turn {attempt}/{MAX_JUNCTION_ATTEMPTS}")
            self.turn_right()
            result = self.advance_and_search(JUNCTION_SEARCH_SECONDS)
            if result == "found":
                return
            if result == "front_blocked":
                # Let the normal front-blocked handling take it from here
                # next tick instead of turning again into a wall.
                return
        print("    gave up finding the right wall after "
              f"{MAX_JUNCTION_ATTEMPTS} turns")

    def avoid_side_wall(self, close_sensor):
        # A side wall this close needs a real stop-and-move-away response,
        # not just driving forward while waiting for the next decision.
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
            # object closer than its minimum range (i.e. touching it). If the
            # last reading was already close, a jump to 255 almost certainly
            # means the wall is now too close to measure, not that it
            # vanished - keep treating it as blocked instead of trusting the
            # raw value.
            if value >= SENSOR_NO_ECHO_RAW and self.history[number]:
                previous = statistics.median(self.history[number])
                if previous < FRONT_SLOWDOWN_CM:
                    value = 0
            self.history[number].append(value)
        return {
            number: statistics.median(values)
            for number, values in self.history.items()
        }

    def run(self):
        print("Right-wall follower running. Press Ctrl+C to stop.")
        started_at = time.monotonic()

        while not self.robot.shutdown_now:
            started = time.monotonic()
            distances = self.read_distances()
            left = distances[SENSOR_LEFT]
            centre = distances[SENSOR_CENTRE]
            right = distances[SENSOR_RIGHT]

            print(
                f"left={left:5.1f}  centre={centre:5.1f}  right={right:5.1f}",
                end="\r",
                flush=True,
            )

            # An exit normally opens into a large clear area. Requiring many
            # samples prevents a single bad ultrasonic reading from ending a
            # run, and the start grace period stops a spacious starting bay
            # from looking identical to the real finish before it's moved.
            if min(left, centre, right) >= EXIT_OPEN_CM:
                self.exit_open_count += 1
            else:
                self.exit_open_count = 0

            past_start_grace = (
                time.monotonic() - started_at >= START_GRACE_SECONDS
            )
            if past_start_grace and self.exit_open_count >= EXIT_CONFIRM_SAMPLES:
                self.stop()
                print("\nOpen finish area detected; robot stopped.")
                return

            # A side wall this close overrides everything else - handle it
            # immediately rather than waiting for the next normal decision.
            if left < SIDE_STOP_CM or right < SIDE_STOP_CM:
                close_sensor = SENSOR_LEFT if left < right else SENSOR_RIGHT
                print(f"\n>>> AVOID {'LEFT' if close_sensor == SENSOR_LEFT else 'RIGHT'} "
                      f"at left={left:.0f} centre={centre:.0f} right={right:.0f}")
                self.avoid_side_wall(close_sensor)
                self.right_open_streak = 0
                self.front_blocked_streak = 0
                continue

            # A junction can show up as a sustained high absolute reading,
            # or as a sudden rise from wherever right was recently tracking
            # even if it doesn't reach RIGHT_OPEN_CM outright.
            right_jumped = (
                len(self.recent_right) == self.recent_right.maxlen
                and right - min(self.recent_right) >= RIGHT_JUMP_CM
            )
            self.recent_right.append(right)

            right_open_now = right >= RIGHT_OPEN_CM or right_jumped
            front_blocked_now = centre <= FRONT_STOP_CM

            self.right_open_streak = (
                self.right_open_streak + 1 if right_open_now else 0
            )
            self.front_blocked_streak = (
                self.front_blocked_streak + 1 if front_blocked_now else 0
            )

            if self.right_open_streak >= RIGHT_OPEN_CONFIRM_SAMPLES:
                print(f"\n>>> JUNCTION at left={left:.0f} "
                      f"centre={centre:.0f} right={right:.0f}")
                self.right_open_streak = 0
                self.front_blocked_streak = 0
                self.recent_right.clear()
                self.navigate_junction()
                continue

            if self.front_blocked_streak >= FRONT_BLOCK_CONFIRM_SAMPLES:
                print(f"\n>>> TURN_LEFT at left={left:.0f} "
                      f"centre={centre:.0f} right={right:.0f}")
                self.right_open_streak = 0
                self.front_blocked_streak = 0
                self.turn_left()
                continue

            # Neither turn is confirmed yet. If the front is genuinely clear
            # right now, keep driving; if it looks blocked but isn't
            # confirmed, pause rather than risk driving into something that
            # might be real.
            if front_blocked_now:
                self.send(0, 0)
            else:
                if centre < FRONT_SLOWDOWN_CM:
                    speed_scale = (centre - FRONT_STOP_CM) / (
                        FRONT_SLOWDOWN_CM - FRONT_STOP_CM
                    )
                    speed_scale = max(0.0, min(1.0, speed_scale))
                    forward_speed = MIN_FORWARD_SPEED + (
                        FORWARD_SPEED - MIN_FORWARD_SPEED
                    ) * speed_scale
                else:
                    forward_speed = FORWARD_SPEED

                # Light, always-on correction so the two motors' mismatch
                # can't drift the heading unopposed. Right holds a true
                # target (always live except at the exact setpoint); left is
                # only a floor - it pushes back if crossed but doesn't fight
                # to hold an exact left distance too.
                right_error = right - RIGHT_TARGET_CM
                left_error = max(0.0, LEFT_MIN_CM - left)
                correction = clamp(
                    (right_error + left_error) * STEER_GAIN,
                    -MAX_STEER_CORRECTION, MAX_STEER_CORRECTION,
                )
                self.send(forward_speed + correction, forward_speed - correction)

            remaining = CONTROL_PERIOD - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    robot = imrt_robot_serial.IMRTRobotSerial()

    try:
        robot.connect(port)
        robot.run()
        RightWallFollower(robot).run()
    except Exception as error:
        print(f"\nRobot program stopped because of an error: {error}")
        raise
    finally:
        # This is best effort: connection failures can happen before a serial
        # port exists, while normal exits should always send an explicit stop.
        if hasattr(robot, "serial_port_"):
            try:
                RightWallFollower(robot).stop()
            except Exception:
                pass
        try:
            # Stops the serial receive thread. Needed on any exit path other
            # than Ctrl+C (e.g. reaching "exit found"), since that thread
            # would otherwise keep the process alive indefinitely.
            robot._shutdown()
        except Exception:
            pass
        print("Robot stopped.")


if __name__ == "__main__":
    main()
