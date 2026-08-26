import imrt_robot_serial
import imrt_xbox
import time


ROBOT_WIDTH = 0.40 # m

# Matches V_MAX in the arduino firmware (motor_serial_arduino.ino). Wheel
# commands above this get clipped by the motor controller, so we scale them
# down ourselves (see below) instead of letting that clipping silently eat
# the turn differential.
MOTOR_CMD_LIMIT = 500

def main():
    wz_gain = 6

    loop_period = 0.1 # s, matches the time.sleep() at the end of the loop

    # m/s, top forward/reverse speed. 2.5 is the fastest the robot can
    # physically go in a straight line given MOTOR_CMD_LIMIT and the 200
    # units-per-m/s scaling below (500 / 200 = 2.5) - going higher here
    # wouldn't do anything on a straight run, only affects how much speed
    # gets traded away for turning at full throttle (see MOTOR_CMD_LIMIT).
    max_speed = 2.5

    # Exponential smoothing factor for the steering command (0-1). Lower
    # values give a smoother/slower turn response, higher values are
    # snappier but more prone to jerks from noise or a shaky hand.
    smoothing_alpha = 0.6

    controller = imrt_xbox.IMRTxbox()

    # Create motor serial object
    motor_serial = imrt_robot_serial.IMRTRobotSerial()


    # Open serial port. Exit if serial port cannot be opened
    try:
        motor_serial.connect("/dev/ttyACM0")
    except:
        print("Could not open port. Is your robot connected?\nExiting program")
        sys.exit()


    # Start serial receive thread
    motor_serial.run()


    wz_smoothed = 0.0

    try:
        while not motor_serial.shutdown_now:
            but_a = controller.get_a()
            but_b = controller.get_b()
            but_x = controller.get_x()
            but_y = controller.get_y()

            ax_lx = controller.get_left_x()
            ax_ly = controller.get_left_y()

            accel_in = controller.get_right_trigger() # 0.0-1.0
            decel_in = controller.get_left_trigger()   # 0.0-1.0

            # Speed tracks trigger press directly: harder press = faster,
            # and it drops back to zero the instant both are released.
            vx = max_speed * (accel_in - decel_in)

            # Left stick steers: how far left/right it's pushed sets the turn rate.
            wz_target = -wz_gain * ax_lx

            # Smooth the steering command so a noisy/jerky stick doesn't
            # translate into a sudden jolt in the motor commands.
            wz_smoothed += smoothing_alpha * (wz_target - wz_smoothed)

            wz = wz_smoothed
            print(vx, wz)

            # calculate motor commands
            v1 = (vx - ROBOT_WIDTH * wz / 2) * 200
            v2 = (vx + ROBOT_WIDTH * wz / 2) * 200

            # If either wheel command would exceed what the motor controller
            # accepts, scale BOTH down together (instead of letting the
            # firmware clip them independently) so the turn differential
            # between the wheels is preserved and we can still turn while
            # driving at high speed.
            largest_cmd = max(abs(v1), abs(v2))
            if largest_cmd > MOTOR_CMD_LIMIT:
                scale = MOTOR_CMD_LIMIT / largest_cmd
                v1 *= scale
                v2 *= scale


            print ("HEI")
            # send motor commands
            motor_serial.send_command(int(v1), int(v2))


            #print("a: {}, b: {}, x: {}, y: {}, lx: {:+.2f}, ly: {:+.2f}, accel: {:.2f}, decel: {:.2f}".format(but_a, but_b, but_x, but_y, ax_lx, ax_ly, accel_in, decel_in), end='\r')

            time.sleep(loop_period)


    finally:
        controller.shutdown()
        print("Exiting program")


if __name__ == '__main__':
    main()

