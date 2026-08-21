# Quick diagnostic: confirms which physical sensor each dist_N number
# actually reads. Run this, then cover each physical ultrasonic sensor with
# your hand one at a time (leave the others uncovered) and watch which
# printed number drops to a small value - that number is the one behind
# whichever sensor your hand is on.
#
# maze.py currently assumes dist_1=right, dist_2=left, dist_3=centre,
# dist_4=behind, copied from labels in imrt_robot_sensor_example.py that
# were never independently confirmed. In the last test log, dist_2 (assumed
# left) read a constant 255 ("no echo") for the entire run, even while
# driving straight into the left wall - so either that channel isn't wired
# to a working sensor, or dist_2 isn't actually the left-facing one.
#
# If covering the physical left sensor doesn't move dist_2 at all, but does
# move a different number, the fix is to update SENSOR_LEFT (and whichever
# other constant now points at the wrong sensor) in maze.py. If nothing
# moves dist_2 no matter which sensor you cover, that channel is a wiring or
# hardware fault, not something fixable in this Python code.

import sys
import time

import imrt_robot_serial


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    robot = imrt_robot_serial.IMRTRobotSerial()
    robot.connect(port)
    robot.run()

    print("Cover each sensor by hand, one at a time. Ctrl+C to stop.")
    try:
        while not robot.shutdown_now:
            print(
                f"dist_1={robot.get_dist_1():3d}  "
                f"dist_2={robot.get_dist_2():3d}  "
                f"dist_3={robot.get_dist_3():3d}  "
                f"dist_4={robot.get_dist_4():3d}",
                end="\r",
                flush=True,
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print("\nDone.")


if __name__ == "__main__":
    main()
