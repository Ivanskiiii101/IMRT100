# Right-hand wall follower for IMRT100.
#
# Hugs the right wall at a target distance band, turns right when a real
# opening in that wall is confirmed, and turns left at a dead end. Built on
# the same hardened patterns proven out in maze.py this session:
#   - sensor mapping confirmed by hand: dist_1=right, dist_2=left,
#     dist_3=centre, dist_4=behind
#   - raw 255 readings right after a close reading are treated as still
#     blocked, not as open space (ultrasonic sensors report 255 both for
#     "nothing in range" and for an object closer than their minimum range)
#   - forward speed never drops below a motor-stall floor
#   - steering correction is never a hard zero, to avoid uncorrected drift
#     from any mismatch between the two motors
#   - front-block and exit detection require several consecutive readings,
#     not one, so a single noisy sample can't trigger a false stop/turn
#   - every drive after a turn keeps checking the front sensor instead of
#     trusting a fixed duration blind - that gap was directly responsible
#     for "turns, then drives straight into a wall" in maze.py
#   - backing up before a turn is live rear-sensor-checked, not a fixed
#     blind reverse
#   - motor_1/motor_2 -> left/right wheel, positive = forward, confirmed
#     directly on the robot with motor_direction_test.py

from collections import deque
import statistics
import sys
import time

import imrt_robot_serial


# Sensor-number mapping, confirmed by hand on the robot.
SENSOR_RIGHT = 1
SENSOR_LEFT = 2
SENSOR_CENTRE = 3
SENSOR_BEHIND = 4

# Motor commands. The Arduino accepts -500 to +500.
FORWARD_SPEED = 135
MIN_FORWARD_SPEED = 90  # below this the motors likely can't overcome friction
TURN_SPEED = 140
BACKUP_SPEED = 120
CORRECTION = 35
INSIDE_BAND_GAIN = 0.4  # gentle pull toward band centre; much weaker than the
                         # gain used outside the band, but never exactly zero -
                         # a hard zero leaves any motor speed mismatch
                         # completely uncorrected, causing slow heading drift.

# Distance thresholds in centimetres; tune these in the real maze.
FRONT_STOP_CM = 35
FRONT_SLOWDOWN_CM = 80
RIGHT_NEAR_CM = 20      # steer away from the wall below this
RIGHT_FAR_CM = 55       # steer back toward the wall above this
RIGHT_OPEN_CM = 75      # sustained reading above this means a real junction
EXIT_OPEN_CM = 180
REAR_CLEARANCE_CM = 15
# A side wall this close needs a real stop-and-move-away response, not just
# a steering nudge while still driving toward it.
SIDE_STOP_CM = 10

# Raw sensor value (0-255) meaning "no echo received." Also happens when an
# object is closer than the sensor's minimum range - see read_distances().
SENSOR_NO_ECHO_RAW = 250

# Timing values that must be calibrated with Venusaur mounted.
CONTROL_PERIOD = 0.08       # 12.5 Hz; safely inside Arduino's 500 ms timeout
TURN_SECONDS = 0.85
JUNCTION_ADVANCE_SECONDS = 0.25
POST_TURN_ADVANCE_SECONDS = 0.35
BACKUP_SECONDS = 0.2
SIDE_AVOID_BACKUP_SECONDS = 0.10
SIDE_AVOID_TURN_SECONDS = 0.15
EXIT_CONFIRM_SAMPLES = 12   # 1.2 seconds of open space
OPEN_CONFIRM_SAMPLES = 3

# Require a couple of consecutive close readings before treating the front
# as genuinely blocked, not just one. A single noisy reading (most likely in
# open space, where there's nothing nearby to actually be causing it) can
# otherwise trigger a stop-and-turn that shouldn't have happened.
FRONT_BLOCK_CONFIRM_SAMPLES = 2

# A spacious starting bay can read just as open as the real finish area on
# every sensor - ignore exit detection for this long after starting, so the
# robot actually gets moving into the maze before it's ever checked.
START_GRACE_SECONDS = 5.0

# After the timed pivot, keep nudging in short bursts if the right sensor
# has not yet found a plausible wall, instead of trusting the fixed timing
# alone (motor speed drifts with battery voltage and floor friction).
PIVOT_CORRECTION_STEP_SECONDS = 0.08
MAX_PIVOT_CORRECTION_SECONDS = 0.4

# If the robot has to turn again multiple times in a row without ever
# driving forward in between, a short nudge back isn't enough to escape a
# tight pocket - back up further, toward the rear wall, to get real room.
STUCK_TURN_THRESHOLD = 2
MAX_ESCAPE_BACKUP_SECONDS = 1.0

# If the left or centre distance drops at least this much from its recent
# minimum, the robot has gotten close to a wall on that side - a strong sign
# it's swinging around a corner as the right wall opens up. Treat that as
# extra confirmation of a real junction, on top of RIGHT_OPEN_CM.
WALL_APPROACH_DROP_CM = 15

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
        self.right_open_count = 0
        self.exit_open_count = 0
        self.front_blocked_count = 0
        # A "right is open" reading only means a junction if we were already
        # hugging a wall. Before the first wall contact (e.g. starting in an
        # open bay) it just means there's nothing there yet, and reflexively
        # turning right in a loop is wrong.
        self.wall_acquired = False
        self.recent_left = deque(maxlen=5)
        self.recent_centre = deque(maxlen=5)
        # Counts consecutive blocked-and-turn events with no forward driving
        # in between - see dead_end_turn().
        self.consecutive_blocked = 0

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

    def back_up_to_wall(self, max_duration):
        # Back up while continuously checking the rear sensor, instead of a
        # single check-then-commit-to-a-fixed-duration drive. If there's
        # room, it backs up until the rear sensor says a wall is actually
        # close; if there isn't, it does nothing rather than reversing blind.
        deadline = time.monotonic() + max_duration
        while time.monotonic() < deadline and not self.robot.shutdown_now:
            behind = self.read_distances()[SENSOR_BEHIND]
            if behind < REAR_CLEARANCE_CM:
                break
            self.send(-BACKUP_SPEED, -BACKUP_SPEED)
            time.sleep(0.05)
        self.stop()

    def turn_right(self):
        self.stop()
        # Move the wheel axle toward the centre of the junction before
        # pivoting - sensor-checked, not blind, so a tight corner can't
        # drive it further into the corner than there's actually room for
        # before it starts turning from a bad position.
        self._advance_while_clear(JUNCTION_ADVANCE_SECONDS, speed=FORWARD_SPEED)
        self._pivot(TURN_SPEED, -TURN_SPEED)
        # At an outside corner, the right sensor often still sees no wall
        # immediately after pivoting (nothing there yet), which would
        # otherwise re-trigger another right turn on the very next reading -
        # several in a row is a full spin in place. Advance into the new
        # corridor first so the right sensor gets a chance to find its wall.
        # This keeps checking the front the whole time instead of driving a
        # fixed duration blind - a turn can end up here either because it
        # found a clear front, or because its correction loop simply ran out
        # of time without ever confirming clear.
        self._advance_while_clear(POST_TURN_ADVANCE_SECONDS)

    def turn_left(self):
        self.stop()
        self._pivot(-TURN_SPEED, TURN_SPEED)
        self._advance_while_clear(POST_TURN_ADVANCE_SECONDS)

    def dead_end_turn(self, behind):
        self.stop()
        self.consecutive_blocked += 1

        if self.consecutive_blocked >= STUCK_TURN_THRESHOLD:
            print("\nStuck in a pocket - backing up further to escape.")
            self.back_up_to_wall(MAX_ESCAPE_BACKUP_SECONDS)
        else:
            self.back_up_to_wall(BACKUP_SECONDS)
        self.turn_left()

    def _advance_while_clear(self, duration, speed=MIN_FORWARD_SPEED):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and not self.robot.shutdown_now:
            centre = self.read_distances()[SENSOR_CENTRE]
            if centre <= FRONT_STOP_CM:
                break
            self.send(speed, speed)
            time.sleep(0.05)
        self.stop()

    def avoid_side_wall(self, close_sensor):
        # A side wall this close needs a real stop-and-move-away response,
        # not just a steering nudge while still driving toward it. Back
        # straight off it a little, then nudge away before resuming.
        self.stop()
        self.timed_drive(-BACKUP_SPEED, -BACKUP_SPEED, SIDE_AVOID_BACKUP_SECONDS)
        if close_sensor == SENSOR_LEFT:
            self.timed_drive(TURN_SPEED, -TURN_SPEED, SIDE_AVOID_TURN_SECONDS)
        else:
            self.timed_drive(-TURN_SPEED, TURN_SPEED, SIDE_AVOID_TURN_SECONDS)
        self.stop()

    def _pivot(self, motor_1, motor_2, duration=TURN_SECONDS):
        self.timed_drive(motor_1, motor_2, duration)
        self.stop()

        # The fixed duration above is only an estimate; verify the right
        # sensor now sees a wall at a plausible distance and, if not, keep
        # rotating in short bursts rather than trusting the timing alone.
        deadline = time.monotonic() + MAX_PIVOT_CORRECTION_SECONDS
        while time.monotonic() < deadline and not self.robot.shutdown_now:
            right = self.read_distances()[SENSOR_RIGHT]
            if RIGHT_NEAR_CM * 0.5 <= right <= RIGHT_OPEN_CM:
                return
            self.timed_drive(motor_1, motor_2, PIVOT_CORRECTION_STEP_SECONDS)
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
            behind = distances[SENSOR_BEHIND]

            print(
                f"left={left:5.1f}  centre={centre:5.1f}  "
                f"right={right:5.1f}  behind={behind:5.1f}",
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

            # Track whether we've actually made contact with a wall yet.
            # Only a wall we were following can "open up" into a junction.
            if right < RIGHT_OPEN_CM:
                self.wall_acquired = True

            # A sharp drop in the left or centre distance is a strong sign
            # we're swinging around a corner as the right wall opens up.
            # Noisy right readings alone often don't sustain above
            # RIGHT_OPEN_CM long enough to confirm a real junction on their
            # own, so use these as corroborating evidence.
            left_confirms_turn = (
                len(self.recent_left) == self.recent_left.maxlen
                and left <= min(self.recent_left) - WALL_APPROACH_DROP_CM
            )
            centre_confirms_turn = (
                len(self.recent_centre) == self.recent_centre.maxlen
                and centre <= min(self.recent_centre) - WALL_APPROACH_DROP_CM
            )
            self.recent_left.append(left)
            self.recent_centre.append(centre)

            # Right-hand priority: turn right whenever a real opening persists
            # in a wall we were already following. Before any wall has been
            # found, an open right reading just means there's nothing there
            # yet - drive forward and let the wall-following correction below
            # steer toward the first wall it finds instead of spinning here.
            right_looks_open = right >= RIGHT_OPEN_CM
            if self.wall_acquired and (
                right_looks_open or left_confirms_turn or centre_confirms_turn
            ):
                self.right_open_count += 1
            else:
                self.right_open_count = 0

            if self.right_open_count >= OPEN_CONFIRM_SAMPLES:
                self.right_open_count = 0
                self.turn_right()
                continue

            # Require a couple of consecutive close readings before treating
            # the front as genuinely blocked, not just one - a single noisy
            # reading could otherwise trigger a stop-and-turn (even a full
            # spin, if it happens twice) that shouldn't have happened.
            if centre <= FRONT_STOP_CM:
                self.front_blocked_count += 1
            else:
                self.front_blocked_count = 0

            if self.front_blocked_count >= FRONT_BLOCK_CONFIRM_SAMPLES:
                self.front_blocked_count = 0
                self.dead_end_turn(behind)
                continue

            if left < SIDE_STOP_CM or right < SIDE_STOP_CM:
                close_sensor = SENSOR_LEFT if left < right else SENSOR_RIGHT
                self.avoid_side_wall(close_sensor)
                continue

            # Made it back to normal forward driving - no longer stuck.
            self.consecutive_blocked = 0

            # Slow down smoothly as the front wall approaches instead of
            # driving at full speed right up to FRONT_STOP_CM. Without this,
            # the robot only reacts once it is already within FRONT_STOP_CM,
            # by which point momentum can carry it into the wall.
            if centre < FRONT_SLOWDOWN_CM:
                speed_scale = (centre - FRONT_STOP_CM) / (
                    FRONT_SLOWDOWN_CM - FRONT_STOP_CM
                )
                speed_scale = max(0.0, min(1.0, speed_scale))
                # Floor at MIN_FORWARD_SPEED, not 0: a command too weak to
                # overcome motor friction leaves the robot stuck reporting
                # the same distance forever instead of actually stopping.
                forward_speed = MIN_FORWARD_SPEED + (
                    FORWARD_SPEED - MIN_FORWARD_SPEED
                ) * speed_scale
            else:
                forward_speed = FORWARD_SPEED

            # Outside the [RIGHT_NEAR_CM, RIGHT_FAR_CM] band, correct firmly.
            # Inside it, still pull gently toward the band centre rather than
            # applying zero correction - a hard zero would leave any inherent
            # motor speed mismatch to accumulate into an uncorrected drift.
            if right < RIGHT_NEAR_CM:
                error = right - RIGHT_NEAR_CM
                gain = 2
            elif right > RIGHT_FAR_CM:
                error = right - RIGHT_FAR_CM
                gain = 2
            else:
                error = right - (RIGHT_NEAR_CM + RIGHT_FAR_CM) / 2
                gain = INSIDE_BAND_GAIN
            correction = clamp(error * gain, -CORRECTION, CORRECTION)
            motor_1 = forward_speed + correction
            motor_2 = forward_speed - correction
            self.send(motor_1, motor_2)

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
