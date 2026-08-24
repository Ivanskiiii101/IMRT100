# speed_run.py - IMRT100 maze solver.
#
# Strategy: right-hand wall following. It's not the shortest possible path
# through the maze, but it's guaranteed to reach the exit as long as the
# maze is simply connected (every wall traces back to the outer boundary,
# true here), and it only needs the three front-facing sensors - no memory
# of the maze layout, no mapping pass. That combination is what makes it
# fast to actually finish a run instead of fast in theory.
#
# Hardware, confirmed by direct testing on this robot (not assumed from
# labels): dist_1 = right sensor, dist_2 = left, dist_3 = front, dist_4 =
# rear - the rear sensor is read but never used for a decision. motor_1 =
# left wheel, motor_2 = right wheel, positive command = forward.
#
# The sensors are noisy - readings jump around with nothing physically
# changing - so every decision here is a named zone confirmed over several
# consecutive ticks, never a single raw reading:
#   - CLOSE/NORMAL/FAR zones on each side drive steering; a reading right
#     at a boundary just moves into the neighbouring zone's behaviour
#   - CLEAR/SLOW/BLOCKED zones on the front drive speed; an unconfirmed
#     BLOCKED reading crawls at the slowdown floor instead of stopping
#     outright, so noise near that boundary can't stall the robot
#   - a junction (right opening up) and a dead end (front closing in) each
#     need to hold for several ticks before being trusted, and picking the
#     wall back up after a turn does too - a single stray reading in any
#     of these was enough to cause a false turn in earlier testing
#   - turning left out of a dead end only stops once several consecutive
#     ticks confirm the front is clear, not one - a single lucky reading
#     stops the turn before the robot has actually turned clear, and the
#     very next tick re-triggers another turn; several of those in a row
#     compound into something that looks like a full spin
#   - turning right at a junction backs away from the wall before pivoting,
#     since that turn fires exactly when the robot is hugging closest to
#     it and can otherwise catch a corner on the wall mid-turn
#   - there is exactly one turn per detected event, never a retry loop -
#     retries that turn further on each attempt are what caused full spins
#     in earlier testing, independent of any single-reading bug
#
# Everything above came from repeated on-robot testing, not guesses.

from collections import deque
import statistics
import sys
import time

import imrt_robot_serial


# --- Hardware mapping (confirmed on this robot) ---------------------------
SENSOR_RIGHT = 1
SENSOR_LEFT = 2
SENSOR_FRONT = 3
SENSOR_REAR = 4  # read every tick, never used for a decision

MOTOR_LEFT_SIGN = 1
MOTOR_RIGHT_SIGN = 1

# --- Speed (Arduino accepts -500..500) -------------------------------------
CRUISE_SPEED = 135
MIN_FORWARD_SPEED = 90  # floor for the slowdown ramp and unconfirmed BLOCKED
TURN_SPEED = 140
BACKUP_SPEED = 120

# --- Front zones -----------------------------------------------------------
FRONT_STOP_CM = 30
FRONT_SLOWDOWN_CM = 80

# --- Right: junction detection is separate from hug-distance tracking,
# since reusing one threshold for both made normal hug variation
# indistinguishable from a real opening. --------------------------------
RIGHT_OPEN_CM = 150
RIGHT_JUMP_CM = 60       # a sudden rise from the recent minimum also counts
RIGHT_REACQUIRE_CM = 90  # counts as "found the wall again" after a turn
RIGHT_TURN_SECONDS = 0.85  # fixed-duration junction pivot - tune on the robot

RIGHT_NEAR_CM = 30
RIGHT_FAR_CM = 50
LEFT_NEAR_CM = 30
LEFT_FAR_CM = 70
LEFT_SENSE_CM = 120  # beyond this there's no left wall to correct toward
STEER_GAIN = 2
INSIDE_BAND_GAIN = 0.4  # gentle pull inside the band - never a hard zero,
                         # or the two motors' mismatch drifts unopposed
MAX_STEER_CORRECTION = 35

SIDE_STOP_CM = 10  # closer than this needs an immediate stop-and-move-away
MAX_CONSECUTIVE_DEAD_ENDS = 4  # safety cap: give up rather than spin forever

EXIT_OPEN_CM = 180
EXIT_CONFIRM_SAMPLES = 12
START_GRACE_SECONDS = 5.0  # a spacious start bay can look like the exit

RIGHT_OPEN_CONFIRM_SAMPLES = 3
FRONT_BLOCK_CONFIRM_SAMPLES = 2
FRONT_CLEAR_CONFIRM_SAMPLES = 3  # symmetric with FRONT_BLOCK_CONFIRM_SAMPLES
REACQUIRE_CONFIRM_SAMPLES = 3

SENSOR_NO_ECHO_RAW = 250  # 255 means "no echo" - also happens when an
                          # object is closer than the sensor's minimum range

CONTROL_PERIOD = 0.08     # 12.5 Hz - inside the Arduino's 500ms timeout
TURN_STEP_SECONDS = 0.05
MAX_TURN_SECONDS = 1.7    # safety cap for the dead-end turn only
SIDE_AVOID_BACKUP_SECONDS = 0.10
SIDE_AVOID_TURN_SECONDS = 0.15


def clamp(value, lower=-500, upper=500):
    return max(lower, min(upper, int(value)))


def classify_front(front):
    if front <= FRONT_STOP_CM:
        return "BLOCKED"
    if front < FRONT_SLOWDOWN_CM:
        return "SLOW"
    return "CLEAR"


def band_contribution(value, near, far, sign):
    # Classifies a side reading into CLOSE/NORMAL/FAR and returns both the
    # zone and the steering contribution. sign=+1 for the right sensor,
    # sign=-1 for the left - same zones, mirrored correction direction.
    mid = (near + far) / 2
    if value < near:
        zone, error, gain = "CLOSE", value - near, STEER_GAIN
    elif value > far:
        zone, error, gain = "FAR", value - far, STEER_GAIN
    else:
        zone, error, gain = "NORMAL", value - mid, INSIDE_BAND_GAIN
    return zone, sign * error * gain


class MazeSolver:
    def __init__(self, robot):
        self.robot = robot
        # maxlen=5: sensors mounted close together can pick up a
        # neighbour's echo for a tick or two - a wider median window needs
        # more than one bad sample in a row to move the result.
        self.history = {number: deque(maxlen=5) for number in (1, 2, 3, 4)}
        self.exit_open_count = 0
        self.right_open_streak = 0
        self.front_blocked_streak = 0
        self.recent_right = deque(maxlen=5)
        # Toggles per turn: True while actively tracking the right wall
        # closely, False while searching for it again after a junction.
        self.wall_acquired = False
        self.reacquire_streak = 0
        # One-way latch, separate from wall_acquired: once true, stays
        # true for the rest of the run. Gates junction detection so the
        # open starting bay can't be misread as "the wall just opened up" -
        # and, unlike wall_acquired, is never reset by a dead-end turn, so
        # one dead end early on can't permanently disable right turns.
        self.has_ever_found_wall = False
        self.consecutive_dead_ends = 0
        self.give_up = False

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

    def turn_right(self):
        self.stop()
        self.timed_drive(-BACKUP_SPEED, -BACKUP_SPEED, SIDE_AVOID_BACKUP_SECONDS)
        self.timed_drive(TURN_SPEED, -TURN_SPEED, RIGHT_TURN_SECONDS)
        self.stop()

    def turn_left(self):
        self.stop()
        self.rotate_until_clear(-TURN_SPEED, TURN_SPEED)

    def avoid_side_wall(self, close_sensor):
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
            # A jump to "no echo" right after a close reading almost always
            # means the object is now too close to measure, not that it
            # vanished - keep treating it as blocked.
            if value >= SENSOR_NO_ECHO_RAW and self.history[number]:
                previous = statistics.median(self.history[number])
                if previous < FRONT_SLOWDOWN_CM:
                    value = 0
            self.history[number].append(value)
        return {
            number: statistics.median(values)
            for number, values in self.history.items()
        }

    def _reset_turn_state(self):
        self.right_open_streak = 0
        self.front_blocked_streak = 0
        self.reacquire_streak = 0
        self.consecutive_dead_ends = 0
        self.recent_right.clear()

    def run(self):
        print("speed_run: maze solver running. Press Ctrl+C to stop.")
        started_at = time.monotonic()

        while not self.robot.shutdown_now and not self.give_up:
            tick_started = time.monotonic()
            distances = self.read_distances()
            left = distances[SENSOR_LEFT]
            front = distances[SENSOR_FRONT]
            right = distances[SENSOR_RIGHT]
            front_zone = classify_front(front)

            print(
                f"left={left:5.1f}  front={front:5.1f} [{front_zone:7s}]  "
                f"right={right:5.1f}",
                end="\r",
                flush=True,
            )

            if min(left, front, right) >= EXIT_OPEN_CM:
                self.exit_open_count += 1
            else:
                self.exit_open_count = 0

            past_start_grace = time.monotonic() - started_at >= START_GRACE_SECONDS
            if past_start_grace and self.exit_open_count >= EXIT_CONFIRM_SAMPLES:
                self.stop()
                print("\n>>> Exit found - wide open on all sides. Stopping.")
                return

            if left < SIDE_STOP_CM or right < SIDE_STOP_CM:
                close_sensor = SENSOR_LEFT if left < right else SENSOR_RIGHT
                side = "LEFT" if close_sensor == SENSOR_LEFT else "RIGHT"
                print(f"\n>>> AVOID {side} at left={left:.0f} right={right:.0f}")
                self.avoid_side_wall(close_sensor)
                self.right_open_streak = 0
                self.front_blocked_streak = 0
                continue

            if right <= RIGHT_REACQUIRE_CM:
                self.has_ever_found_wall = True

            right_jumped = (
                self.has_ever_found_wall
                and len(self.recent_right) == self.recent_right.maxlen
                and right - min(self.recent_right) >= RIGHT_JUMP_CM
            )
            self.recent_right.append(right)

            right_open_now = self.has_ever_found_wall and (
                right >= RIGHT_OPEN_CM or right_jumped
            )
            front_blocked_now = front_zone == "BLOCKED"

            self.right_open_streak = (
                self.right_open_streak + 1 if right_open_now else 0
            )
            self.front_blocked_streak = (
                self.front_blocked_streak + 1 if front_blocked_now else 0
            )

            if self.right_open_streak >= RIGHT_OPEN_CONFIRM_SAMPLES:
                print(f"\n>>> TURN RIGHT (junction) left={left:.0f} "
                      f"front={front:.0f} right={right:.0f}")
                self._reset_turn_state()
                self.turn_right()
                self.wall_acquired = False
                continue

            if self.front_blocked_streak >= FRONT_BLOCK_CONFIRM_SAMPLES:
                if right_open_now:
                    # Front usually confirms first purely because it needs
                    # fewer samples - trust a strong right-open reading
                    # immediately once we're at a real decision point,
                    # rather than waiting out its own full streak.
                    print(f"\n>>> TURN RIGHT (junction, front closing) "
                          f"left={left:.0f} front={front:.0f} right={right:.0f}")
                    self._reset_turn_state()
                    self.turn_right()
                    self.wall_acquired = False
                    continue
                self.consecutive_dead_ends += 1
                if self.consecutive_dead_ends > MAX_CONSECUTIVE_DEAD_ENDS:
                    self.stop()
                    self.give_up = True
                    print(f"\n>>> Stopping: {MAX_CONSECUTIVE_DEAD_ENDS} dead "
                          "ends in a row with no progress.")
                    continue
                print(f"\n>>> TURN LEFT (dead end "
                      f"{self.consecutive_dead_ends}/{MAX_CONSECUTIVE_DEAD_ENDS}) "
                      f"left={left:.0f} front={front:.0f} right={right:.0f}")
                self.right_open_streak = 0
                self.front_blocked_streak = 0
                self.turn_left()
                continue

            if not self.wall_acquired:
                if right <= RIGHT_REACQUIRE_CM:
                    self.reacquire_streak += 1
                else:
                    self.reacquire_streak = 0
                if self.reacquire_streak >= REACQUIRE_CONFIRM_SAMPLES:
                    print(f"\n<<< wall reacquired at right={right:.0f}")
                    self.wall_acquired = True
                    self.reacquire_streak = 0
                    self.recent_right.clear()

            self.consecutive_dead_ends = 0

            if front_zone == "CLEAR":
                forward_speed = CRUISE_SPEED
            elif front_zone == "SLOW":
                scale = (front - FRONT_STOP_CM) / (FRONT_SLOWDOWN_CM - FRONT_STOP_CM)
                scale = max(0.0, min(1.0, scale))
                forward_speed = MIN_FORWARD_SPEED + (
                    CRUISE_SPEED - MIN_FORWARD_SPEED
                ) * scale
            else:  # BLOCKED but not yet confirmed - crawl, don't stop
                forward_speed = MIN_FORWARD_SPEED

            if left >= LEFT_SENSE_CM:
                left_contribution = 0.0
            else:
                _, left_contribution = band_contribution(
                    left, LEFT_NEAR_CM, LEFT_FAR_CM, sign=-1
                )

            if self.wall_acquired:
                _, right_contribution = band_contribution(
                    right, RIGHT_NEAR_CM, RIGHT_FAR_CM, sign=1
                )
            elif right < RIGHT_NEAR_CM:
                # Not tracking yet, but close enough to need a push away
                # regardless - see SIDE_STOP_CM in the module docstring.
                _, right_contribution = band_contribution(
                    right, RIGHT_NEAR_CM, RIGHT_FAR_CM, sign=1
                )
            else:
                right_contribution = 0.0

            correction = clamp(
                right_contribution + left_contribution,
                -MAX_STEER_CORRECTION, MAX_STEER_CORRECTION,
            )
            self.send(forward_speed + correction, forward_speed - correction)

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
