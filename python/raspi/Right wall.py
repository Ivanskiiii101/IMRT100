# Right-hand wall follower for IMRT100.
#
# Decision logic keeps the requested 3-sensor state machine (front/right/
# left only - the rear sensor is intentionally never used for any decision)
# exactly as given:
#
#   front_blocked = sensor_front < threshold
#   right_blocked = sensor_right < threshold
#   not right_blocked      -> TURN_RIGHT
#   not front_blocked      -> MOVE_FORWARD
#   front_blocked and right_blocked -> TURN_LEFT
#   else                   -> U_TURN (structurally unreachable, kept as a
#                              defensive fallback)
#
# Two things were added on top, both informed by extensive testing this
# project, without changing that structure:
#   - front and right get their own threshold instead of one shared value -
#     testing repeatedly found the front needs more stopping margin
#     (~28-35cm) than the right-hug distance (~20cm) ever did
#   - TURN_RIGHT/TURN_LEFT require the trigger to hold for a few consecutive
#     ticks before being acted on (see run()) - reacting to a single sensor
#     reading was directly responsible for false turns and full U-turns in
#     earlier testing. MOVE_FORWARD needs no such debounce since it's the
#     safe, reversible default; while a possible front-block is still
#     unconfirmed, the robot pauses rather than driving forward into
#     something that might be real - a genuine use for the "else" branch,
#     which the original algorithm never reaches at all.
#
# Everything else here is just how each action is physically carried out:
#   - sensor mapping confirmed by hand: dist_1=right, dist_2=left,
#     dist_3=centre, dist_4=behind (behind is read but unused)
#   - raw 255 readings right after a close reading are treated as still
#     blocked, not as open space (ultrasonic sensors report 255 both for
#     "nothing in range" and for an object closer than their minimum range)
#   - forward speed eases off approaching the front threshold instead of
#     holding full speed to the last centimetre - "hits the wall too hard"
#     was a repeated, concrete complaint in testing
#   - a turn rotates in small increments, checking the front after each one,
#     instead of committing to a fixed duration for a specific angle
#   - motor_1/motor_2 -> left/right wheel, positive = forward, confirmed
#     directly on the robot with motor_direction_test.py

from collections import deque
import statistics
import sys
import time

import imrt_robot_serial


SENSOR_RIGHT = 1
SENSOR_LEFT = 2
SENSOR_CENTRE = 3
SENSOR_BEHIND = 4  # read but never used for a decision

# Motor commands. The Arduino accepts -500 to +500.
FORWARD_SPEED = 135
MIN_FORWARD_SPEED = 90  # floor for the slowdown ramp - below this the
                         # motors likely can't overcome friction
TURN_SPEED = 140

# Distance in centimetres below which a sensor counts as "a wall is there."
# front_blocked/right_blocked in the original algorithm both used one shared
# wall_threshold (0.20m); split here per the testing note above.
FRONT_THRESHOLD_CM = 30
RIGHT_THRESHOLD_CM = 20
# Start easing forward speed down from this far out, so it's already slow
# by the time FRONT_THRESHOLD_CM is reached instead of braking abruptly.
FRONT_SLOWDOWN_CM = 80

# Require the trigger to hold for this many consecutive control-loop ticks
# before actually committing to a turn - see the module docstring.
RIGHT_OPEN_CONFIRM_SAMPLES = 3
FRONT_BLOCK_CONFIRM_SAMPLES = 2

EXIT_OPEN_CM = 180
EXIT_CONFIRM_SAMPLES = 12   # 1.2 seconds of open space
# A spacious starting bay can read just as open as the real finish area on
# every sensor - ignore exit detection for this long after starting.
START_GRACE_SECONDS = 5.0

# Raw sensor value (0-255) meaning "no echo received." Also happens when an
# object is closer than the sensor's minimum range - see read_distances().
# This and RECENT_CLOSE_CM only clean up raw sensor noise before it's
# compared to the thresholds above; unrelated to the decision algorithm.
SENSOR_NO_ECHO_RAW = 250
RECENT_CLOSE_CM = 80

CONTROL_PERIOD = 0.08       # 12.5 Hz; safely inside Arduino's 500 ms timeout
TURN_STEP_SECONDS = 0.05
MAX_TURN_SECONDS = 1.7      # safety cap per turn (covers roughly 180 degrees)

# Change either sign if a positive command drives that motor backwards.
# Confirmed correct (motor_1=left, motor_2=right, positive=forward) with
# motor_direction_test.py.
MOTOR_1_SIGN = 1
MOTOR_2_SIGN = 1


def clamp(value, lower=-500, upper=500):
    return max(lower, min(upper, int(value)))


def follow_right_wall(sensor_front, sensor_right, sensor_left):
    # Same structure and priority as originally given, just with the split
    # thresholds explained above.
    front_blocked = sensor_front < FRONT_THRESHOLD_CM
    right_blocked = sensor_right < RIGHT_THRESHOLD_CM

    if not right_blocked:
        return "TURN_RIGHT"
    elif not front_blocked:
        return "MOVE_FORWARD"
    elif front_blocked and right_blocked:
        return "TURN_LEFT"
    else:
        return "U_TURN"


class RightWallFollower:
    def __init__(self, robot):
        self.robot = robot
        # maxlen=5: with sensors mounted close together, one can occasionally
        # pick up a neighbour's echo (crosstalk) and report a bogus close
        # reading for a tick or two. A wider median window needs more than
        # one or two bad-in-a-row samples to actually move the result.
        self.history = {number: deque(maxlen=5) for number in (1, 2, 3, 4)}
        self.exit_open_count = 0
        # Consecutive ticks the raw algorithm has suggested each turn - see
        # the module docstring for why these gate acting on it.
        self.right_open_streak = 0
        self.front_blocked_streak = 0

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

    def rotate_until_clear(self, motor_1, motor_2):
        deadline = time.monotonic() + MAX_TURN_SECONDS
        while time.monotonic() < deadline and not self.robot.shutdown_now:
            self.send(motor_1, motor_2)
            time.sleep(TURN_STEP_SECONDS)
            centre = self.read_distances()[SENSOR_CENTRE]
            if centre >= FRONT_THRESHOLD_CM:
                break
        self.stop()

    def turn_right(self):
        self.stop()
        self.rotate_until_clear(TURN_SPEED, -TURN_SPEED)

    def turn_left(self):
        self.stop()
        self.rotate_until_clear(-TURN_SPEED, TURN_SPEED)

    def u_turn(self):
        # Structurally unreachable from follow_right_wall (one of the first
        # three branches always matches given a boolean front/right_blocked),
        # kept only as a defensive fallback.
        self.stop()
        self.rotate_until_clear(TURN_SPEED, -TURN_SPEED)
        self.rotate_until_clear(TURN_SPEED, -TURN_SPEED)

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
                if previous < RECENT_CLOSE_CM:
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

            # The raw algorithm's suggestion, unchanged from what was given.
            raw_action = follow_right_wall(centre, right, left)

            self.right_open_streak = (
                self.right_open_streak + 1 if raw_action == "TURN_RIGHT" else 0
            )
            self.front_blocked_streak = (
                self.front_blocked_streak + 1 if centre < FRONT_THRESHOLD_CM
                else 0
            )

            if raw_action == "TURN_RIGHT" and self.right_open_streak < RIGHT_OPEN_CONFIRM_SAMPLES:
                # Not confirmed yet - keep doing whatever's safe right now
                # rather than committing to a turn off one reading.
                action = "MOVE_FORWARD" if centre >= FRONT_THRESHOLD_CM else "PAUSE"
            elif raw_action == "TURN_LEFT" and self.front_blocked_streak < FRONT_BLOCK_CONFIRM_SAMPLES:
                action = "PAUSE"
            else:
                action = raw_action

            if action == "TURN_RIGHT":
                print(f"\n>>> TURN_RIGHT at left={left:.0f} "
                      f"centre={centre:.0f} right={right:.0f}")
                self.right_open_streak = 0
                self.front_blocked_streak = 0
                self.turn_right()
            elif action == "TURN_LEFT":
                print(f"\n>>> TURN_LEFT at left={left:.0f} "
                      f"centre={centre:.0f} right={right:.0f}")
                self.right_open_streak = 0
                self.front_blocked_streak = 0
                self.turn_left()
            elif action == "U_TURN":
                print(f"\n>>> U_TURN at left={left:.0f} "
                      f"centre={centre:.0f} right={right:.0f}")
                self.right_open_streak = 0
                self.front_blocked_streak = 0
                self.u_turn()
            elif action == "PAUSE":
                self.send(0, 0)
            else:  # MOVE_FORWARD
                # Ease off speed approaching the front threshold instead of
                # holding full speed to the last centimetre.
                if centre < FRONT_SLOWDOWN_CM:
                    speed_scale = (centre - FRONT_THRESHOLD_CM) / (
                        FRONT_SLOWDOWN_CM - FRONT_THRESHOLD_CM
                    )
                    speed_scale = max(0.0, min(1.0, speed_scale))
                    forward_speed = MIN_FORWARD_SPEED + (
                        FORWARD_SPEED - MIN_FORWARD_SPEED
                    ) * speed_scale
                else:
                    forward_speed = FORWARD_SPEED
                self.send(forward_speed, forward_speed)

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
