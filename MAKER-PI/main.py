#  Tony Smoragiewicz
#  Robotic Car Project
#  July 26 2024

import neopixel
import simpleio
import asyncio
import time
import board
import busio

from sensors import (
    ImuSensor,
    Battery,
    Wheel,
    calibrate_motor,
    compute_motor_commands,
    robot_tipped_over,
    initialize_motors,
    initialize_buttons,
    TofSensor,
)
from math_utils import map_ranges, angle_from_acceleration, sign
from helper_functions import print_number_list
from definitions import *

import supervisor

supervisor.runtime.autoreload = False

uart = busio.UART(TX0, RX0, baudrate=BAUD_RATE, timeout=0)

# tof = TofSensor()
imu = ImuSensor()
# battery = Battery()
motor1, motor2 = initialize_motors()
btn1, btn2 = initialize_buttons()
wheel = Wheel()

# Initialize Neopixel RGB LEDs
pixels = neopixel.NeoPixel(NEOPIXEL_PIN, 2)
pixels.fill(0)


# -------------------------------------------------
# ON START: Show running light and play melody
# -------------------------------------------------
for i in range(len(MELODY_NOTE)):
    simpleio.tone(PIEZO_PIN, MELODY_NOTE[i], duration=MELODY_DURATION[i])


def main():
    while True:
        time.sleep(0.5)
        motor1.throttle = None
        motor2.throttle = None

        start_time = time.monotonic()

        for i in range(10):
            measurements = imu.read()
            print_number_list(measurements)

        t, acc_y, _, _, _, _, _, _, _, _ = measurements
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

            u = compute_motor_commands(omega, theta, sum_torque)
            sgn = sign(u)

            u1 = map_ranges(u, input_range=(0, sgn), output_range=(sgn * 0.38, sgn))
            u2 = map_ranges(u, input_range=(0, sgn), output_range=(sgn * 0.30, sgn))

            u1, u2 = 0.0, 0.0
            motor1.throttle = u2
            motor2.throttle = u1

            measurements[0] = t
            string = print_number_list(measurements + [u1, u2], decimal=2)
            uart.write(bytes(f"<B,{string}>", "ascii"))

            # tof.read()

            if robot_tipped_over(acc_y):
                motor1.throttle = None
                motor2.throttle = None
                print("RESET")
                time.sleep(1.0)
                break


if __name__ == "__main__":
    main()
