# Quick diagnostic: confirms which physical wheel each motor number drives,
# and which command sign makes that wheel spin forwards. Run this with the
# robot up on a stand (wheels off the ground) so it can't drive off a table.
#
# Right wall.py assumes motor_1 = left wheel, motor_2 = right wheel, and that
# a positive command drives each wheel forwards. This script isolates each
# motor one at a time so you can watch and confirm - or find out that's
# wrong, in which case set MOTOR_1_SIGN/MOTOR_2_SIGN or swap the mapping in
# Right wall.py accordingly.

import sys
import time

import imrt_robot_serial

TEST_SPEED = 150
DRIVE_SECONDS = 1.0
PAUSE_SECONDS = 1.5


def drive_one_motor(robot, label, cmd_1, cmd_2):
    print(f"\n{label}: watch the robot now...")
    end_time = time.monotonic() + DRIVE_SECONDS
    while time.monotonic() < end_time and not robot.shutdown_now:
        robot.send_command(cmd_1, cmd_2)
        time.sleep(0.05)
    robot.send_command(0, 0)
    time.sleep(PAUSE_SECONDS)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    robot = imrt_robot_serial.IMRTRobotSerial()
    robot.connect(port)
    robot.run()

    print("Put the robot on a stand with wheels off the ground.")
    print("Press Enter when ready...")
    input()

    drive_one_motor(robot, "motor_1 positive (expected: LEFT wheel, forward)",
                     TEST_SPEED, 0)
    drive_one_motor(robot, "motor_2 positive (expected: RIGHT wheel, forward)",
                     0, TEST_SPEED)

    print("\nDone. For each step, note which wheel actually spun and "
          "which direction (forward/backward) it turned.")
    robot.close()


if __name__ == "__main__":
    main()
