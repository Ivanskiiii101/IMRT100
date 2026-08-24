# Maze navigator for IMRT100.
#
# Not a right-hand-wall-follower: this drives down the middle of the
# corridor using whichever side sensors currently see a wall, slows and
# stops before hitting anything in front, and turns toward whichever side
# looks more open when blocked. Carries forward everything learned testing
# "Right wall.py":
#   - sensor mapping matches imrt_robot_sensor_example.py's labels
#   - raw 255 readings right after a close reading are treated as still
#     blocked, not as open space (ultrasonic sensors report 255 both for
#     "nothing in range" and for an object closer than their minimum range)
#   - forward speed never drops below a motor-stall floor
#   - steering correction is never a hard zero, to avoid uncorrected drift
#     from any mismatch between the two motors
#   - motor_1/motor_2 -> left/right wheel, positive = forward, was confirmed
#     directly on the robot with motor_direction_test.py

from collections import deque
import statistics
import sys
import threading
import time

import imrt_robot_serial
import RPi.GPIO as GPIO


# Sensor-number mapping, matching the labels used in
# imrt_robot_sensor_example.py. Re-verify by covering each sensor by hand if
# the robot still seems to misjudge distances.
SENSOR_RIGHT = 1
SENSOR_LEFT = 2
SENSOR_CENTRE = 3
SENSOR_BEHIND = 4

# Motor commands. The Arduino accepts -500 to +500.
FORWARD_SPEED = 135  # was 150, cut another 10% - stopping was too slow and
                     # getting too close to the wall
MIN_FORWARD_SPEED = 90  # below this the motors likely can't overcome friction
TURN_SPEED = 140
BACKUP_SPEED = 120
STEER_GAIN = 2.5
MAX_STEER_CORRECTION = 45

# Distance thresholds in centimetres; tune these in the real maze.
FRONT_STOP_CM = 35      # raised from 28 - triggers the stop/turn sooner,
                         # more margin before actually reaching the wall
FRONT_SLOWDOWN_CM = 80   # raised to match - braking starts earlier and more
                         # gradually, instead of a late, abrupt slowdown
SIDE_TOO_CLOSE_CM = 25    # steer away if only one wall is this close - raised
                         # so it reacts sooner now that the robot covers more
                         # ground per control-loop tick at the higher speed
CORRIDOR_SENSE_CM = 80    # ignore side readings farther than this for centring
# Gentle steering correction alone can't always avoid a wall that's already
# this close - it's a shallow arc while still moving forward, not an
# emergency response. Below this, stop and physically move away instead of
# just steering while still driving toward it.
SIDE_STOP_CM = 10
EXIT_OPEN_CM = 180
REAR_CLEARANCE_CM = 15

# Raw sensor value (0-255) meaning "no echo received." Also happens when an
# object is closer than the sensor's minimum range - see read_distances().
SENSOR_NO_ECHO_RAW = 250

# Timing values that must be calibrated with Venusaur mounted.
CONTROL_PERIOD = 0.08       # 12.5 Hz; safely inside Arduino's 500 ms timeout.
                             # Tighter than 0.10 since more distance is
                             # covered per tick at the higher FORWARD_SPEED.
POST_TURN_ADVANCE_SECONDS = 0.35
BACKUP_SECONDS = 0.2
SIDE_AVOID_BACKUP_SECONDS = 0.10  # timed_drive resends every 0.05s, so this
                                   # is 2 backward pulses instead of 3
SIDE_AVOID_TURN_SECONDS = 0.15

# A side sensor is most likely to miss a wall at a glancing angle right
# after a turn, before the centring correction has had time to straighten
# the heading out. Cap speed for a bit after every turn so that correction
# gets more time/distance to work before trusting full speed again.
TURN_COOLDOWN_SECONDS = 1.0
TURN_COOLDOWN_SPEED = MIN_FORWARD_SPEED + 30
EXIT_CONFIRM_SAMPLES = 12   # 1.2 seconds of open space
# A spacious starting bay can read just as open as the real finish area on
# every sensor - ignore exit detection for this long after starting, so the
# robot actually gets moving into the maze before it's ever checked.
START_GRACE_SECONDS = 5.0

# Require a couple of consecutive close readings before treating the front
# as genuinely blocked, not just one. A single noisy reading (most likely in
# open space, where there's nothing nearby to actually be causing it) can
# otherwise trigger a stop-and-turn that shouldn't have happened.
FRONT_BLOCK_CONFIRM_SAMPLES = 2

# turn() rotates in small increments and checks the front after each one,
# instead of committing to a fixed duration for a specific angle. This lets
# it handle corners that aren't 90 degrees - it just keeps turning until the
# front is actually clear rather than assuming how far that takes.
# MAX_TURN_SECONDS is only a safety cap (covers roughly a full 180 degrees).
TURN_STEP_SECONDS = 0.05
MAX_TURN_SECONDS = 1.7

# If the robot has to turn again multiple times in a row without ever
# driving forward in between, a short nudge back isn't enough to escape a
# tight pocket - back up further, toward the rear wall, to get real room.
STUCK_TURN_THRESHOLD = 2
MAX_ESCAPE_BACKUP_SECONDS = 1.0

# Change either sign if a positive command drives that motor backwards.
# Confirmed correct (motor_1=left, motor_2=right, positive=forward) with
# motor_direction_test.py.
MOTOR_1_SIGN = 1
MOTOR_2_SIGN = 1

# Piezo buzzer wired straight to a Raspberry Pi GPIO pin (see
# gpio_tune_player.py) - independent of the Arduino/motor serial link, so
# playing music doesn't touch anything on the Arduino side.
BUZZ_PIN = 23
BUZZ_DUTY = 10

c = [32, 65, 131, 262, 523]
d = [36, 73, 147, 294, 587]
e = [41, 82, 165, 330, 659]
f = [43, 87, 175, 349, 698]
g = [49, 98, 196, 392, 784]
a = [55, 110, 220, 440, 880]

WHOLE = 0.8
HALF = WHOLE / 2
QUART = WHOLE / 4

# (pitch, duration) pairs - swap this out for any other tune.
SONG = list(zip(
    [c[3], d[3], e[3], f[3], g[3], g[3], a[3], a[3],
     a[3], a[3], g[3], f[3], f[3], f[3], f[3], e[3],
     e[3], d[3], d[3], d[3], d[3], c[3]],
    [QUART, QUART, QUART, QUART, HALF, HALF, QUART, QUART,
     QUART, QUART, WHOLE, QUART, QUART, QUART, QUART, HALF,
     HALF, QUART, QUART, QUART, QUART, WHOLE],
))


def clamp(value, lower=-500, upper=500):
    return max(lower, min(upper, int(value)))


class MusicPlayer:
    # Plays SONG on loop on a background thread, driving the buzzer with
    # PWM, so it doesn't block the sensor/motor control loop.
    def __init__(self, song, pin=BUZZ_PIN):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, 250)
        self._pwm.start(0)
        self._song = song
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._play_loop, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join()
        self._pwm.stop()
        GPIO.cleanup()

    def _play_loop(self):
        while not self._stop_event.is_set():
            for pitch, duration in self._song:
                if self._stop_event.is_set():
                    break
                self._pwm.ChangeDutyCycle(BUZZ_DUTY)
                self._pwm.ChangeFrequency(pitch)
                self._interruptible_sleep(duration / 2)
                self._pwm.ChangeDutyCycle(0)
                self._interruptible_sleep(duration / 2)

    def _interruptible_sleep(self, duration):
        # Sleep in small steps so stop() doesn't have to wait out a note.
        end_time = time.monotonic() + duration
        while not self._stop_event.is_set():
            remaining = end_time - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(0.05, remaining))


class MazeNavigator:
    def __init__(self, robot):
        self.robot = robot
        # maxlen=5: with sensors mounted close together, one can occasionally
        # pick up a neighbour's echo (crosstalk) and report a bogus close
        # reading for a tick or two. A wider median window needs more than
        # one or two bad-in-a-row samples to actually move the result.
        self.history = {number: deque(maxlen=5) for number in (1, 2, 3, 4)}
        self.exit_open_count = 0
        self.front_blocked_count = 0
        # Counts consecutive blocked-and-turn events with no forward driving
        # in between - see handle_blocked().
        self.consecutive_blocked = 0
        # monotonic() timestamp until which forward speed stays capped after
        # a turn - see turn() and the speed calculation in run().
        self.turn_cooldown_until = 0.0

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

    def turn(self, motor_1, motor_2):
        # Rotate in small increments, checking the front after each one,
        # instead of committing to a fixed duration for a specific angle.
        # This naturally handles corners that aren't 90 degrees - it just
        # keeps turning until the front is actually clear, whatever angle
        # that takes, up to the MAX_TURN_SECONDS safety cap.
        deadline = time.monotonic() + MAX_TURN_SECONDS
        while time.monotonic() < deadline and not self.robot.shutdown_now:
            self.send(motor_1, motor_2)
            time.sleep(TURN_STEP_SECONDS)
            centre = self.read_distances()[SENSOR_CENTRE]
            if centre > FRONT_STOP_CM:
                break
        self.stop()

        # Ease forward, but keep checking the front the whole time. The
        # rotation above can end up here either because it found a clear
        # front, or because it simply ran out of MAX_TURN_SECONDS without
        # ever confirming clear - and even a "clear" reading can be
        # marginal. Don't drive this blind into whatever might still be there.
        advance_deadline = time.monotonic() + POST_TURN_ADVANCE_SECONDS
        while time.monotonic() < advance_deadline and not self.robot.shutdown_now:
            centre = self.read_distances()[SENSOR_CENTRE]
            if centre <= FRONT_STOP_CM:
                break
            self.send(MIN_FORWARD_SPEED, MIN_FORWARD_SPEED)
            time.sleep(0.05)
        self.stop()
        self.turn_cooldown_until = time.monotonic() + TURN_COOLDOWN_SECONDS

    def back_up_to_wall(self, max_duration):
        # Back up while continuously checking the rear sensor, instead of a
        # single check-then-commit-to-a-fixed-duration drive. If there's
        # room, it backs up until the rear sensor says a wall is actually
        # close; if there isn't, it does nothing rather than reversing
        # blind. Side sensors can't be trusted to catch a collision mid-turn
        # (they scan the wall at a constantly-changing angle while spinning,
        # and can miss it entirely), so real clearance has to come from
        # here, before the turn starts, not from watching the sides during it.
        deadline = time.monotonic() + max_duration
        while time.monotonic() < deadline and not self.robot.shutdown_now:
            behind = self.read_distances()[SENSOR_BEHIND]
            if behind < REAR_CLEARANCE_CM:
                break
            self.send(-BACKUP_SPEED, -BACKUP_SPEED)
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

    def handle_blocked(self):
        self.stop()
        self.consecutive_blocked += 1

        if self.consecutive_blocked >= STUCK_TURN_THRESHOLD:
            print("\nStuck in a pocket - backing up further to escape.")
            self.back_up_to_wall(MAX_ESCAPE_BACKUP_SECONDS)
        else:
            self.back_up_to_wall(BACKUP_SECONDS)
        distances = self.read_distances()
        left = distances[SENSOR_LEFT]
        right = distances[SENSOR_RIGHT]

        # Turn toward whichever side currently looks more open.
        if right >= left:
            self.turn(TURN_SPEED, -TURN_SPEED)
        else:
            self.turn(-TURN_SPEED, TURN_SPEED)

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
        print("Maze navigator running. Press Ctrl+C to stop.")
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
            # samples prevents a single bad ultrasonic reading from ending a run.
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

            if centre <= FRONT_STOP_CM:
                self.front_blocked_count += 1
            else:
                self.front_blocked_count = 0

            if self.front_blocked_count >= FRONT_BLOCK_CONFIRM_SAMPLES:
                self.front_blocked_count = 0
                self.handle_blocked()
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

            # Cap speed for a bit after a turn - see TURN_COOLDOWN_SECONDS.
            if time.monotonic() < self.turn_cooldown_until:
                forward_speed = min(forward_speed, TURN_COOLDOWN_SPEED)

            # Steer to stay roughly centred in the corridor. If both walls
            # are visible, balance the distance to each. If only one is,
            # steer away from it once it gets too close. This is never a
            # hard zero - a small mismatch between the two motors would
            # otherwise go uncorrected and drift the heading over time.
            left_near = left < CORRIDOR_SENSE_CM
            right_near = right < CORRIDOR_SENSE_CM
            if left_near and right_near:
                error = right - left
            elif right_near and right < SIDE_TOO_CLOSE_CM:
                error = right - SIDE_TOO_CLOSE_CM
            elif left_near and left < SIDE_TOO_CLOSE_CM:
                error = SIDE_TOO_CLOSE_CM - left
            else:
                error = 0

            correction = clamp(error * STEER_GAIN,
                               -MAX_STEER_CORRECTION, MAX_STEER_CORRECTION)
            motor_1 = forward_speed + correction
            motor_2 = forward_speed - correction
            self.send(motor_1, motor_2)

            remaining = CONTROL_PERIOD - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    robot = imrt_robot_serial.IMRTRobotSerial()
    music = None

    try:
        robot.connect(port)
        robot.run()
        try:
            # Music is a nice-to-have; a buzzer/GPIO problem (e.g. needing
            # to run with sudo) must never stop the robot from driving.
            music = MusicPlayer(SONG)
            music.start()
        except Exception as music_error:
            print(f"\nMusic unavailable ({music_error}); driving without it.")
            music = None
        MazeNavigator(robot).run()
    except Exception as error:
        print(f"\nRobot program stopped because of an error: {error}")
        raise
    finally:
        if music is not None:
            try:
                music.stop()
            except Exception:
                pass
        # This is best effort: connection failures can happen before a serial
        # port exists, while normal exits should always send an explicit stop.
        if hasattr(robot, "serial_port_"):
            try:
                MazeNavigator(robot).stop()
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
