from collections import deque
import statistics
import sys
import time

import imrt_robot_serial


# Sensor-number mapping, matching the labels used in
# imrt_robot_sensor_example.py. Re-verify by covering each sensor by hand if
# the robot still seems to misjudge distances.
SENSOR_RIGHT = 1
SENSOR_LEFT = 2
SENSOR_CENTRE = 3
SENSOR_BEHIND = 4

# Conservative initial motor commands. The Arduino accepts -500 to +500.
FORWARD_SPEED = 150
CORRECTION = 35
TURN_SPEED = 140

# Initial distance thresholds in centimetres; tune these in the real maze.
FRONT_STOP_CM = 25
RIGHT_TARGET_CM = 22
RIGHT_OPEN_CM = 55
EXIT_OPEN_CM = 180

# Timing values that must be calibrated with Venusaur mounted.
CONTROL_PERIOD = 0.10       # 10 Hz; safely inside Arduino's 500 ms timeout
TURN_90_SECONDS = 0.85
JUNCTION_ADVANCE_SECONDS = 0.25
EXIT_CONFIRM_SAMPLES = 12   # 1.2 seconds of open space
OPEN_CONFIRM_SAMPLES = 3

# After the timed pivot, keep nudging in short bursts if the right sensor
# has not yet found a plausible wall, instead of trusting the fixed timing
# alone (motor speed drifts with battery voltage and floor friction).
PIVOT_CORRECTION_STEP_SECONDS = 0.08
MAX_PIVOT_CORRECTION_SECONDS = 0.4

# Before pivoting in place at a dead end, back away from the front wall a
# little if the rear sensor confirms there is room, so the chassis corners
# don't clip the wall while turning.
REAR_CLEARANCE_CM = 15
BACKUP_SPEED = 120
BACKUP_SECONDS = 0.2

# Change either sign if a positive command drives that motor backwards.
MOTOR_1_SIGN = 1
MOTOR_2_SIGN = 1


def clamp(value, lower=-500, upper=500):
    return max(lower, min(upper, int(value)))


class RightWallFollower:
    def __init__(self, robot):
        self.robot = robot
        self.history = {number: deque(maxlen=3) for number in (1, 2, 3, 4)}
        self.right_open_count = 0
        self.exit_open_count = 0

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

    def turn_right(self):
        self.stop()
        # Move the wheel axle toward the centre of the junction before pivoting.
        self.timed_drive(FORWARD_SPEED, FORWARD_SPEED,
                         JUNCTION_ADVANCE_SECONDS)
        self._pivot(TURN_SPEED, -TURN_SPEED)

    def turn_left(self):
        self.stop()
        self._pivot(-TURN_SPEED, TURN_SPEED)

    def dead_end_turn(self, behind):
        self.stop()
        # Only back up if the rear sensor confirms there's room; otherwise
        # pivot in place as before.
        if behind >= REAR_CLEARANCE_CM:
            self.timed_drive(-BACKUP_SPEED, -BACKUP_SPEED, BACKUP_SECONDS)
        self.turn_left()

    def _pivot(self, motor_1, motor_2):
        self.timed_drive(motor_1, motor_2, TURN_90_SECONDS)
        self.stop()

        # The fixed duration above is only an estimate; verify the right
        # sensor now sees a wall at a plausible distance and, if not, keep
        # rotating in short bursts rather than trusting the timing alone.
        deadline = time.monotonic() + MAX_PIVOT_CORRECTION_SECONDS
        while time.monotonic() < deadline and not self.robot.shutdown_now:
            right = self.read_distances()[SENSOR_RIGHT]
            if RIGHT_TARGET_CM * 0.5 <= right <= RIGHT_OPEN_CM:
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
            self.history[number].append(value)
        return {
            number: statistics.median(values)
            for number, values in self.history.items()
        }

    def run(self):
        print("Right-wall follower running. Press Ctrl+C to stop.")

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
            # samples prevents a single bad ultrasonic reading from ending a run.
            if min(left, centre, right) >= EXIT_OPEN_CM:
                self.exit_open_count += 1
            else:
                self.exit_open_count = 0

            if self.exit_open_count >= EXIT_CONFIRM_SAMPLES:
                self.stop()
                print("\nOpen finish area detected; robot stopped.")
                return

            # Right-hand priority: turn right whenever a real opening persists.
            if right >= RIGHT_OPEN_CM:
                self.right_open_count += 1
            else:
                self.right_open_count = 0

            if self.right_open_count >= OPEN_CONFIRM_SAMPLES:
                self.right_open_count = 0
                self.turn_right()
                continue

            # If forward is blocked and right was not open, follow the wall by
            # turning left. Repeating this at a dead end produces a U-turn.
            if centre <= FRONT_STOP_CM:
                self.dead_end_turn(behind)
                continue

            # While moving straight, gently correct toward the target distance
            # from the right wall. This assumes the right sensor sees that wall.
            error = right - RIGHT_TARGET_CM
            correction = clamp(error * 2, -CORRECTION, CORRECTION)
            motor_1 = FORWARD_SPEED + correction
            motor_2 = FORWARD_SPEED - correction
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
            robot.close()
        except Exception:
            pass
        print("Robot stopped.")


if __name__ == "__main__":
    main()
