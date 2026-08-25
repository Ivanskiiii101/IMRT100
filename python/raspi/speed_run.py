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
# The finish jingle is a single blocking call, not a background thread:
# it only ever runs once the robot has already stopped for good, so
# there's no control loop left for a thread to avoid blocking.

from collections import deque
import statistics
import sys
import time

import imrt_robot_serial
import RPi.GPIO as GPIO


SENSOR_RIGHT = 1
SENSOR_LEFT = 2
SENSOR_FRONT = 3
SENSOR_REAR = 4  # read every tick, never used for a decision

MOTOR_LEFT_SIGN = 1
MOTOR_RIGHT_SIGN = 1

CRUISE_SPEED = 150      # the one speed dial - bump this to go faster
TURN_SPEED = 140
BACKUP_SPEED = 120

FRONT_STOP_CM = 35
# A bit more clearance than a dead-ahead stop needs: with no slowdown
# ramp, there's nothing nudging the robot away from a side wall early -
# this is the only side defence there is.
SIDE_STOP_CM = 12
NO_ECHO_RECOVERY_CM = 80  # see the 255-sentinel handling in read_distances()

FRONT_BLOCK_CONFIRM_SAMPLES = 2
FRONT_CLEAR_CONFIRM_SAMPLES = 3  # symmetric with the above - see docstring

BACKUP_SECONDS = 0.2
STUCK_BACKUP_SECONDS = 0.6   # a bigger backup once bouncing repeatedly
STUCK_THRESHOLD = 2          # with no forward progress in between
MAX_CONSECUTIVE_STUCK = 6    # give up rather than bounce forever

SIDE_AVOID_BACKUP_SECONDS = 0.10
SIDE_AVOID_TURN_SECONDS = 0.15

TURN_STEP_SECONDS = 0.05
MAX_TURN_SECONDS = 1.7  # safety cap only - see rotate_until_clear()

EXIT_OPEN_CM = 180
EXIT_CONFIRM_SAMPLES = 12
START_GRACE_SECONDS = 5.0  # a spacious start bay can look like the exit

SENSOR_NO_ECHO_RAW = 250
CONTROL_PERIOD = 0.06

# Piezo buzzer wired straight to a Raspberry Pi GPIO pin (see
# gpio_tune_player.py) - independent of the Arduino/motor serial link, so
# playing a sound can never touch anything on the Arduino side.
BUZZ_PIN = 23
BUZZ_DUTY = 10
BEEP_PITCH = 440  # a plain, audible "A" - not a tune, just a chirp
# Long enough to actually hear, short enough that even if it somehow
# overran a tick or two, it would be negligible - but it never blocks, so
# in practice it doesn't cost the control loop any time at all.
BEEP_SECONDS = 0.12
# Cue the beep when this many of the 3 sensors read blocked at once - a
# tight squeeze, not necessarily a stop-and-turn.
BEEP_BLOCKED_SENSORS = 2


def clamp(value, lower=-500, upper=500):
    return max(lower, min(upper, int(value)))


def setup_buzzer(pin=BUZZ_PIN):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 250)
    pwm.start(0)
    return pwm


class MazeSolver:
    def __init__(self, robot):
        self.robot = robot
        self.history = {number: deque(maxlen=5) for number in (1, 2, 3, 4)}
        self.exit_open_count = 0
        self.front_blocked_streak = 0
        # Consecutive bounces with no successful forward driving in
        # between - not reset until a normal driving tick happens.
        self.consecutive_blocked = 0
        # Buzzer: set up lazily on the first beep, not here - a throwaway
        # MazeSolver used only to send a stop command should never touch
        # GPIO. self.beep_off_at is also the "currently beeping" flag: 0.0
        # means idle, a future timestamp means on until then.
        self.pwm = None
        self.buzzer_failed = False
        self.beep_off_at = 0.0

    def beep(self):
        # Non-blocking: turns the tone on now and returns immediately.
        # update_beep(), called every tick from run(), turns it off again
        # on a later tick - no sleep anywhere in this path, so it can
        # never delay a sensor read or a motor command.
        if self.pwm is None and not self.buzzer_failed:
            try:
                self.pwm = setup_buzzer()
            except Exception as error:
                print(f"(sound unavailable: {error})")
                self.buzzer_failed = True
        if self.pwm is None:
            return
        self.pwm.ChangeDutyCycle(BUZZ_DUTY)
        self.pwm.ChangeFrequency(BEEP_PITCH)
        self.beep_off_at = time.monotonic() + BEEP_SECONDS

    def update_beep(self):
        if self.beep_off_at and time.monotonic() >= self.beep_off_at:
            self.pwm.ChangeDutyCycle(0)
            self.beep_off_at = 0.0

    def close_buzzer(self):
        if self.pwm is not None:
            self.pwm.stop()
            GPIO.cleanup()

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

            self.update_beep()  # non-blocking - see beep()/update_beep()
            blocked_sensors = sum((
                front <= FRONT_STOP_CM,
                left <= SIDE_STOP_CM,
                right <= SIDE_STOP_CM,
            ))
            if blocked_sensors >= BEEP_BLOCKED_SENSORS and not self.beep_off_at:
                self.beep()

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

            if left < SIDE_STOP_CM or right < SIDE_STOP_CM:
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
    solver = None

    try:
        robot.connect(port)
        robot.run()
        solver = MazeSolver(robot)
        solver.run()
    except Exception as error:
        print(f"\nRobot program stopped because of an error: {error}")
        raise
    finally:
        if solver is not None:
            try:
                solver.close_buzzer()
            except Exception:
                pass
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
