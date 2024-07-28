#  Tony Smoragiewicz
#  Robotic Car Project
#  July 26 2024

import neopixel
import time

from sensors import (
    ImuSensor,
    robot_tipped_over,
    calibrate_sensors,
)
from sounds import play_start_up_tune

from math_utils import map_ranges, angle_from_acceleration, sign
from helper_functions import print_number_list
from definitions import (
    NEOPIXEL_PIN,
)

from uart_utils import read_from_serial, write_to_serial, initialize_uart

from button_utils import initialize_buttons
from motor_utils import initialize_motors, calibrate_motor, compute_motor_commands


uart = initialize_uart()

imu = ImuSensor()
motor1, motor2 = initialize_motors()
btn1, btn2 = initialize_buttons()

# Initialize Neopixel RGB LEDs
pixels = neopixel.NeoPixel(NEOPIXEL_PIN, 2)
pixels.fill(0)

play_start_up_tune()


def main():
    while True:
        time.sleep(0.5)
        motor1.throttle = None
        motor2.throttle = None

        start_time = time.monotonic()

        calibrate_sensors()

        t, acc_y, _, _, _, _, _, _, _, _ = imu.read()
        theta = angle_from_acceleration(acc_y)

        now = time.monotonic() - start_time
        sum_torque = 0

        while True:
            if not btn1.value:
                calibrate_motor(motor1)
                calibrate_motor(motor2)

            measurements = imu.read()

            t, acc_y, acc_z, omega, _, _, _, _, _, _ = measurements
            t -= start_time

            dt = t - now
            now = t

            theta += omega * dt

            message = read_from_serial(uart)
            if message is not None:
                u1, u2 = message
                u1 = float(u1)
                u2 = float(u2)

            else:
                u = compute_motor_commands(omega, theta, sum_torque)
                sgn = sign(u)

                u1 = map_ranges(u, input_range=(0, sgn), output_range=(sgn * 0.38, sgn))
                u2 = map_ranges(u, input_range=(0, sgn), output_range=(sgn * 0.30, sgn))

                u1, u2 = 0.0, 0.0
            motor1.throttle = u2
            motor2.throttle = u1

            measurements[0] = t
            string = print_number_list(measurements + [u1, u2], decimal=2)
            write_to_serial(uart, message=string)

            if robot_tipped_over(acc_y):
                motor1.throttle = None
                motor2.throttle = None
                print("RESET")
                time.sleep(1.0)
                break


if __name__ == "__main__":
    main()
