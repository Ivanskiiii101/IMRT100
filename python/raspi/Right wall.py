# Right-hand wall follower for IMRT100.
#
# Three sensors, kept deliberately simple:
#   - keep the right wall within an acceptable range, not one fixed number
#     (RIGHT_NEAR_CM to RIGHT_FAR_CM)
#   - keep the left wall at bay the same way (LEFT_NEAR_CM to LEFT_FAR_CM)
#   - if the right reading changes drastically, that's a turn: turn right
#     once, then watch the front - if it reads open, keep moving forward
#     (still keeping left at bay) until the right wall is picked up again,
#     at which point go back to tracking it closely as normal
#   - if the front is blocked, it's a dead end: turn left
#
# Two bugs fixed after the last few tests:
#   - junction detection (turning right) was gated by wall_acquired, the
#     same flag that tracks the CURRENT steering mode. That flag goes False
#     after every junction turn and stays False until the wall is picked
#     back up - but a dead-end TURN_LEFT never touches it. So if a dead end
#     was ever hit while wall_acquired happened to be False (e.g. right at
#     the start, before any wall had been found yet), right-turn detection
#     was silently disabled for the rest of the run - every future blocked
#     front then fell through to TURN_LEFT with no way to ever turn right
#     again, which compounds into exactly the kind of repeated-left-turn
#     spin that walks the robot back the way it came. Junction detection is
#     now gated by a separate one-way flag, has_ever_found_wall, that only
#     needs to become True once, ever, and is never affected by turns.
#   - there was also a multi-attempt "turn, search, if not found turn
#     again" loop here. Three retries, each turning further, compounded
#     into a full spin on its own. There is now exactly one turn per
#     junction - no retry, no escalation.
#
# Turn detection and hug-distance steering are still two separate concerns
# with two separate sets of numbers, since reusing one right-side threshold
# for both broke the same way every time earlier - normal hug-distance
# variation looked identical to a real opening. RIGHT_OPEN_CM/RIGHT_JUMP_CM
# only decide "has the wall genuinely changed"; RIGHT_NEAR_CM/RIGHT_FAR_CM/
# LEFT_NEAR_CM/LEFT_FAR_CM only decide how to steer while driving straight.
#
# These sensors are noisy - readings jump around even with nothing physically
# changing - so every raw threshold comparison has been replaced with a named
# zone (classify_front, band_contribution). The zones exist for two reasons:
#   - a reading right at a boundary should move you into the neighbouring
#     zone's behaviour, not trigger a one-off special case for that boundary
#   - moving between zones is cheap to reason about and print, so it's
#     obvious from the live output which zone actually drove any given
#     decision, instead of re-deriving it from raw numbers after the fact
# This is also what fixed a real bug: front used to hard-stop
# (self.send(0, 0)) on any single tick where centre dipped to FRONT_STOP_CM,
# well before FRONT_BLOCK_CONFIRM_SAMPLES had confirmed it was a real dead
# end - a noisy reading oscillating near that boundary meant full motor
# stops over and over through open corridors ("stops 100 times"). An
# unconfirmed BLOCKED front now just means "crawl at MIN_FORWARD_SPEED",
# the same as the bottom of the SLOW zone - only a *confirmed* BLOCKED
# streak (still gated by FRONT_BLOCK_CONFIRM_SAMPLES, unchanged) actually
# stops the robot to turn.
#
# A real test log also showed this: right after a junction turn (still
# searching, wall not yet reacquired), right drifted to 8cm - under
# SIDE_STOP_CM - and needed three AVOID_RIGHT emergency calls in a row to
# get clear. Cause: search steering only ever reacted to left, by design,
# so nothing opposed the right side getting that close until the hard
# emergency stop, which isn't always enough in one try. Right now also
# pushes away (same CLOSE-zone correction as normal tracking) if it drifts
# inside RIGHT_NEAR_CM while searching - still zero at FAR/NORMAL, so a
# still-distant right doesn't fight the search itself.
#
# Another real test showed TURN_RIGHT firing correctly (right had jumped to
# a genuine opening) but the robot just kept driving straight afterward
# instead of turning - it drove on down the old corridor until it hit the
# wall at the end. turn_right() is a blind, fixed-duration pivot with no
# verification that the robot actually turned, unlike turn_left(), which
# keeps checking the front sensor while it rotates. Right had been hugging
# 14-15cm - tighter than the normal tracking band - right up until the
# turn fired, meaning the pivot started with the chassis already close
# enough to the wall to catch a corner on it: the motors still get
# commanded for the full RIGHT_TURN_SECONDS regardless, but a caught corner
# means the heading doesn't actually change. turn_right() now backs up
# briefly first, the same defensive move avoid_side_wall() already uses,
# so the pivot always starts with clearance.
#
# That backup fix alone didn't stop the reported "does a 360 and ends up
# back at the same spot" - because that particular failure was never in
# turn_right() at all. It was rotate_until_clear(), which turn_left() (the
# dead-end recovery) uses: it stopped rotating on a single raw reading
# >=FRONT_STOP_CM, with no confirm-samples debounce at all, unlike every
# other decision in this file. One noisy "clear" tick mid-rotation was
# enough to end the turn early, before the robot had actually turned clear
# - so the next control-loop tick immediately redetected BLOCKED and fired
# another dead-end turn. Several of these small under-rotations in a row
# add up to something close to a full 360, ending up facing back the way
# it came - exactly the reported symptom. Now requires
# FRONT_CLEAR_CONFIRM_SAMPLES consecutive clear ticks before it stops,
# symmetric with how BLOCKED itself is entered.
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
#   - the dead-end turn rotates in small increments, checking the front
#     after each one - meaningful there since the front was genuinely
#     blocked to start with. The junction turn is a fixed duration instead,
#     since the front is usually already clear at a junction, which made
#     "rotate until front clears" fire almost immediately or, with a
#     minimum-time floor added, over-rotate - see RIGHT_TURN_SECONDS.

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
# picked the wall back up - well below RIGHT_OPEN_CM, comfortably above the
# normal RIGHT_NEAR_CM-RIGHT_FAR_CM tracking range below.
RIGHT_REACQUIRE_CM = 90
# Fixed duration for the junction turn - tune this on the robot. Not
# condition-based (see module docstring for why "rotate until front clears"
# doesn't work well for this specific turn).
RIGHT_TURN_SECONDS = 0.85

# The two motors aren't perfectly matched - proven repeatedly earlier in
# this project - so driving with literally equal motor speeds lets the
# robot drift steadily toward one wall with nothing to counteract it.
# Each side has an acceptable range, not one fixed number: inside it, only
# a gentle pull toward the middle (never a hard zero - a true zero is what
# let the drift go uncorrected before); outside it, a firm correction back
# in. Both ranges are kept well clear of RIGHT_OPEN_CM so neither can be
# confused with a real opening.
RIGHT_NEAR_CM = 30      # steer away from the right wall below this
RIGHT_FAR_CM = 50        # steer back toward the right wall above this
LEFT_NEAR_CM = 30       # steer away from the left wall below this
LEFT_FAR_CM = 70         # steer back toward the left wall above this
# Left always contributes to steering (unlike right, it's not gated by
# wall_acquired), so it needs its own cutoff: beyond this, there's no left
# wall there at all to correct toward - most obviously in the open starting
# bay - and treating "no wall" the same as "drifted away from one" was
# commanding a hard, continuous steer toward nothing, right from tick one.
LEFT_SENSE_CM = 120
STEER_GAIN = 2
INSIDE_BAND_GAIN = 0.4
MAX_STEER_CORRECTION = 35

# A side wall this close needs a real stop-and-move-away response, not just
# waiting for the next decision tick.
SIDE_STOP_CM = 10

# Safety net: if a dead-end (TURN_LEFT) fires this many times in a row with
# no successful forward driving in between, stop instead of continuing to
# turn - repeated dead-end turns compounding into a full spin that walks
# the robot back the way it came is the single worst failure mode seen
# testing this, and it's cheap to guard against outright regardless of
# what ends up causing it.
MAX_CONSECUTIVE_DEAD_ENDS = 4

EXIT_OPEN_CM = 180
EXIT_CONFIRM_SAMPLES = 12   # 1.2 seconds of open space
# A spacious starting bay can read just as open as the real finish area on
# every sensor - ignore exit detection for this long after starting.
START_GRACE_SECONDS = 5.0

# Require the trigger to hold for this many consecutive control-loop ticks
# before actually committing to a turn.
RIGHT_OPEN_CONFIRM_SAMPLES = 3
FRONT_BLOCK_CONFIRM_SAMPLES = 2
# Same idea for picking the wall back up after a junction turn - a single
# reading was flipping wall_acquired straight back to True, right when the
# heading is least settled and a stray reading is most likely.
REACQUIRE_CONFIRM_SAMPLES = 3
# Same idea again for rotate_until_clear() ending a dead-end turn - a single
# raw reading was letting the turn stop rotating on one noisy "clear" tick,
# well before the robot had actually turned clear of the obstruction. The
# very next control-loop tick then immediately redetected BLOCKED and fired
# another dead-end turn - repeated small under-rotations compounding into
# something close to a full 360, ending up facing back the way it came.
FRONT_CLEAR_CONFIRM_SAMPLES = 3

# Raw sensor value (0-255) meaning "no echo received." Also happens when an
# object is closer than the sensor's minimum range - see read_distances().
SENSOR_NO_ECHO_RAW = 250

CONTROL_PERIOD = 0.08       # 12.5 Hz; safely inside Arduino's 500 ms timeout
TURN_STEP_SECONDS = 0.05
# Safety cap for the dead-end turn only (covers roughly 180 degrees). The
# junction turn is fixed-duration - see RIGHT_TURN_SECONDS.
MAX_TURN_SECONDS = 1.7
SIDE_AVOID_BACKUP_SECONDS = 0.10
SIDE_AVOID_TURN_SECONDS = 0.15

# Change either sign if a positive command drives that motor backwards.
# Confirmed correct (motor_1=left, motor_2=right, positive=forward) with
# motor_direction_test.py.
MOTOR_1_SIGN = 1
MOTOR_2_SIGN = 1


def clamp(value, lower=-500, upper=500):
    return max(lower, min(upper, int(value)))


def classify_front(centre):
    # Named zones instead of a bare threshold comparison, mainly so the
    # boundary between them can be a smooth speed change (see run()) rather
    # than a hard action triggered by one noisy sample. These sensors jitter
    # near a boundary; reacting instantly to a single BLOCKED reading was
    # sending a full motor stop on every such tick, well before the 2-sample
    # confirmation had a chance to decide whether it was real.
    if centre <= FRONT_STOP_CM:
        return "BLOCKED"
    if centre < FRONT_SLOWDOWN_CM:
        return "SLOW"
    return "CLEAR"


def band_contribution(value, near, far, sign):
    # Classifies a side reading into CLOSE/NORMAL/FAR and returns both the
    # zone and the steering contribution for it. sign=+1 for the right
    # sensor, sign=-1 for the left (same zones, mirrored correction
    # direction - see the two call sites in run()).
    mid = (near + far) / 2
    if value < near:
        zone, error, gain = "CLOSE", value - near, STEER_GAIN
    elif value > far:
        zone, error, gain = "FAR", value - far, STEER_GAIN
    else:
        zone, error, gain = "NORMAL", value - mid, INSIDE_BAND_GAIN
    return zone, sign * error * gain


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
        # True while actively tracking the right wall closely (chase the
        # RIGHT_NEAR_CM-RIGHT_FAR_CM band). False right after a junction
        # turn, until the wall is picked up again - during that time
        # steering only keeps the left wall at bay. This toggles back and
        # forth freely and does NOT gate junction detection below - it only
        # decides how to steer.
        self.wall_acquired = False
        self.reacquire_streak = 0
        # Separate, one-way flag: once the robot has found a wall a single
        # time, this stays True for the rest of the run. This is what gates
        # junction detection, specifically so the open starting bay (no
        # wall found yet) can't be misread as "the wall just opened up."
        # Using wall_acquired for that instead was the actual bug: a dead-
        # end TURN_LEFT never touches wall_acquired, so if one ever fired
        # while it was False, right-turn detection was disabled permanently
        # for the rest of the run - every future blocked front then fell
        # through to TURN_LEFT with no way to ever turn right again.
        self.has_ever_found_wall = False
        # Counts consecutive TURN_LEFT (dead end) turns with no successful
        # forward driving in between - see MAX_CONSECUTIVE_DEAD_ENDS.
        self.consecutive_dead_ends = 0
        # Set when that cap is hit - stops run() rather than continuing to
        # turn indefinitely.
        self.give_up = False

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
        # Meaningful for the dead-end turn: the front was genuinely blocked
        # to start with, so "front clears" is a real signal of having
        # turned away from the obstruction - but only once it holds for
        # several ticks, not one reading, symmetric with how BLOCKED itself
        # is entered (FRONT_BLOCK_CONFIRM_SAMPLES). A single noisy "clear"
        # tick mid-rotation was stopping the turn before the robot had
        # actually turned clear, so the very next tick immediately
        # redetected BLOCKED and fired another dead-end turn - see
        # FRONT_CLEAR_CONFIRM_SAMPLES.
        deadline = time.monotonic() + MAX_TURN_SECONDS
        clear_streak = 0
        while time.monotonic() < deadline and not self.robot.shutdown_now:
            self.send(motor_1, motor_2)
            time.sleep(TURN_STEP_SECONDS)
            centre = self.read_distances()[SENSOR_CENTRE]
            clear_streak = clear_streak + 1 if centre >= FRONT_STOP_CM else 0
            if clear_streak >= FRONT_CLEAR_CONFIRM_SAMPLES:
                break
        self.stop()

    def turn_right(self):
        # Fixed duration, not condition-based - see module docstring. Backs
        # up first: by definition the robot has been hugging the right wall
        # right up until this fires, so pivoting immediately can catch a
        # chassis corner on that same wall. The motors still get commanded
        # for the full RIGHT_TURN_SECONDS either way - this has no
        # verification that the robot actually turned, unlike turn_left,
        # which keeps checking the front sensor while it rotates - so a
        # caught corner means the robot silently keeps facing the old
        # corridor and just drives on down it. A real test showed exactly
        # this: TURN_RIGHT fired right after right had been reading 14-15cm
        # (tighter than the normal tracking band) for a while, and the
        # robot kept driving straight afterward instead of turning.
        self.stop()
        self.timed_drive(-BACKUP_SPEED, -BACKUP_SPEED, SIDE_AVOID_BACKUP_SECONDS)
        self.timed_drive(TURN_SPEED, -TURN_SPEED, RIGHT_TURN_SECONDS)
        self.stop()

    def turn_left(self):
        self.stop()
        self.rotate_until_clear(-TURN_SPEED, TURN_SPEED)

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

        while not self.robot.shutdown_now and not self.give_up:
            started = time.monotonic()
            distances = self.read_distances()
            left = distances[SENSOR_LEFT]
            centre = distances[SENSOR_CENTRE]
            right = distances[SENSOR_RIGHT]
            front_zone = classify_front(centre)

            print(
                f"left={left:5.1f}  centre={centre:5.1f} [{front_zone:7s}]  "
                f"right={right:5.1f}",
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

            # A one-way latch: once a wall's been found, junction detection
            # stays live for the rest of the run regardless of the current
            # steering mode (wall_acquired) - see the note in __init__ for
            # why using wall_acquired itself for this was the actual bug.
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
                print(f"\n>>> TURN_RIGHT (junction) at left={left:.0f} "
                      f"centre={centre:.0f} right={right:.0f}")
                self.right_open_streak = 0
                self.front_blocked_streak = 0
                self.reacquire_streak = 0
                self.consecutive_dead_ends = 0
                self.recent_right.clear()
                self.turn_right()
                # Now search: keep left at bay, don't chase a right target
                # until the wall is actually found again.
                self.wall_acquired = False
                continue

            if self.front_blocked_streak >= FRONT_BLOCK_CONFIRM_SAMPLES:
                if right_open_now:
                    # The front confirmed first purely because it needs
                    # fewer samples (FRONT_BLOCK_CONFIRM_SAMPLES <
                    # RIGHT_OPEN_CONFIRM_SAMPLES), not because this is
                    # actually a dead end - a real right-turn junction
                    # naturally has the front closing in at the same time
                    # the right opens up, so front almost always wins that
                    # race. Once we're definitely at a decision point (front
                    # blocked), trust a single strong right-open reading
                    # immediately instead of waiting for its own full streak.
                    print(f"\n>>> TURN_RIGHT (junction, front also closing) "
                          f"at left={left:.0f} centre={centre:.0f} "
                          f"right={right:.0f}")
                    self.right_open_streak = 0
                    self.front_blocked_streak = 0
                    self.reacquire_streak = 0
                    self.consecutive_dead_ends = 0
                    self.recent_right.clear()
                    self.turn_right()
                    self.wall_acquired = False
                    continue
                self.consecutive_dead_ends += 1
                if self.consecutive_dead_ends > MAX_CONSECUTIVE_DEAD_ENDS:
                    self.stop()
                    self.give_up = True
                    print(
                        "\n>>> Stopping: "
                        f"{MAX_CONSECUTIVE_DEAD_ENDS} dead-end turns in a "
                        "row with no forward progress."
                    )
                    continue
                print(f"\n>>> TURN_LEFT (dead end, "
                      f"{self.consecutive_dead_ends}/{MAX_CONSECUTIVE_DEAD_ENDS}) "
                      f"at left={left:.0f} centre={centre:.0f} right={right:.0f}")
                self.right_open_streak = 0
                self.front_blocked_streak = 0
                self.turn_left()
                continue

            # While searching (wall not yet acquired), picking the wall back
            # up switches steering back to tracking it closely - but only
            # once that holds for several ticks, not one reading. The
            # heading is least settled right after a turn, so a single
            # stray close reading here was flipping tracking back on before
            # the robot was actually facing the new wall, and the resulting
            # correction could itself swing right enough to look like a
            # fresh "jump" - triggering an unwanted second turn.
            if not self.wall_acquired:
                if right <= RIGHT_REACQUIRE_CM:
                    self.reacquire_streak += 1
                else:
                    self.reacquire_streak = 0
                if self.reacquire_streak >= REACQUIRE_CONFIRM_SAMPLES:
                    print(f"\n<<< right wall picked up again at right={right:.0f}")
                    self.wall_acquired = True
                    self.reacquire_streak = 0
                    self.recent_right.clear()

            # Neither turn is confirmed yet. front_zone only sets how fast to
            # go here - it never stops the robot outright. A single BLOCKED
            # reading isn't trusted on its own (that needs
            # FRONT_BLOCK_CONFIRM_SAMPLES, handled above); treated the same
            # way, it just means "crawl at the slowdown floor," same as the
            # bottom of the SLOW zone. Hard-stopping on every unconfirmed
            # BLOCKED tick - which sensor noise flips in and out of
            # constantly - is what made the robot stop and restart over and
            # over instead of driving smoothly through open space.
            self.consecutive_dead_ends = 0

            if front_zone == "CLEAR":
                forward_speed = FORWARD_SPEED
            elif front_zone == "SLOW":
                speed_scale = (centre - FRONT_STOP_CM) / (
                    FRONT_SLOWDOWN_CM - FRONT_STOP_CM
                )
                speed_scale = max(0.0, min(1.0, speed_scale))
                forward_speed = MIN_FORWARD_SPEED + (
                    FORWARD_SPEED - MIN_FORWARD_SPEED
                ) * speed_scale
            else:  # BLOCKED, but not yet confirmed - crawl, don't slam to a stop
                forward_speed = MIN_FORWARD_SPEED

            # Light, always-on correction so the two motors' mismatch can't
            # drift the heading unopposed. Left always contributes (an
            # acceptable range, not a fixed number - see LEFT_NEAR_CM/
            # LEFT_FAR_CM) as long as there's actually a wall there to react
            # to (LEFT_SENSE_CM) - open space, most obviously the starting
            # bay, is not "drifted away from a wall," it's just open, and
            # gets zone NONE / zero correction. Right only contributes while
            # actually tracking the wall; while searching, only left
            # matters, so a still-far right reading doesn't fight the
            # search.
            if left >= LEFT_SENSE_CM:
                left_zone, left_contribution = "NONE", 0.0
            else:
                left_zone, left_contribution = band_contribution(
                    left, LEFT_NEAR_CM, LEFT_FAR_CM, sign=-1
                )

            if self.wall_acquired:
                right_zone, right_contribution = band_contribution(
                    right, RIGHT_NEAR_CM, RIGHT_FAR_CM, sign=1
                )
            elif right < RIGHT_NEAR_CM:
                # Not tracking the wall yet, but right has drifted
                # dangerously close anyway - proven to happen right after a
                # junction turn, before the wall's been picked back up.
                # Normal search steering only reacts to left, by design, so
                # nothing was opposing this until SIDE_STOP_CM's hard
                # emergency stop - which isn't always enough in one try (a
                # real test needed three AVOID_RIGHT calls in a row here).
                # Reuse the same CLOSE-zone push-away right tracking already
                # uses once acquired; FAR/NORMAL still get zero so a
                # still-distant right doesn't fight the search.
                right_zone, right_contribution = band_contribution(
                    right, RIGHT_NEAR_CM, RIGHT_FAR_CM, sign=1
                )
            else:
                right_zone, right_contribution = "SEARCHING", 0.0

            correction = clamp(
                right_contribution + left_contribution,
                -MAX_STEER_CORRECTION, MAX_STEER_CORRECTION,
            )
            self.send(forward_speed + correction, forward_speed - correction)

            # Overwrites the same status line printed at the top of the loop
            # (still \r, not \n) now that the side zones are known - so it's
            # visible live which zone actually drove this tick's steering,
            # not just the raw numbers.
            print(
                f"left={left:5.1f}[{left_zone:9s}]  centre={centre:5.1f} "
                f"[{front_zone:7s}]  right={right:5.1f}[{right_zone:9s}]  "
                f"speed={forward_speed:.0f}",
                end="\r",
                flush=True,
            )

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
