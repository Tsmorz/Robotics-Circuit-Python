import board
import busio
import time
from analogio import AnalogIn
import digitalio
import asyncio

import pwmio
from adafruit_motor import servo, motor

from ulab import numpy as np

from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL

import adafruit_vl53l1x

from definitions import (
    SCL0,
    SDA0,
    SCL1,
    SDA1,
    BATTERY,
    BATTERY_VOLTAGE,
    ENCODER_PINS,
    WHEEL_DIAMETER,
    ENCODER_TICKS_PER_ROTATION,
    GEAR_RATIO,
)


class TofSensor:
    def __init__(self, scl=SCL1, sda=SDA1):
        i2c = busio.I2C(scl, sda)
        self.tof = adafruit_vl53l1x.VL53L1X(i2c)
        self.tof.distance_mode = 1
        self.tof.timing_budget = 20

        print("VL53L1X Simple Test.")
        print("--------------------")
        model_id, module_type, mask_rev = self.tof.model_info
        print("Model ID: 0x{:0X}".format(model_id))
        print("Module Type: 0x{:0X}".format(module_type))
        print("Mask Revision: 0x{:0X}".format(mask_rev))
        print("Distance Mode: ", end="")
        if self.tof.distance_mode == 1:
            print("SHORT")
        elif self.tof.distance_mode == 2:
            print("LONG")
        else:
            print("UNKNOWN")
        print("Timing Budget: {}".format(self.tof.timing_budget))
        print("--------------------")

        self.tof.start_ranging()

        return

    def read(self):
        if self.tof.data_ready:
            print("Distance: {} cm".format(self.tof.distance), time.monotonic())
            self.tof.clear_interrupt()
        return


class ImuSensor:
    def __init__(self, scl=SCL0, sda=SDA0):

        i2c = busio.I2C(scl, sda)
        self.accel_gyro = LSM6DS(i2c)
        self.mag = LIS3MDL(i2c)
        return

    def read(self):
        readings = [time.monotonic()]
        for r in self.accel_gyro.acceleration:
            readings.append(r)
        for r in self.accel_gyro.gyro:
            readings.append(r)
        for r in self.mag.magnetic:
            readings.append(r)
        return readings


class Battery:
    def __init__(self, pin=BATTERY):
        self.pin = AnalogIn(pin)

    def get_voltage(self):
        voltage = self.pin.value * BATTERY_VOLTAGE / 65536
        return voltage


class Wheel:
    """
    Wheel Class
    """

    codes = [
        (True, True),
        (False, True),
        (False, False),
        (True, False),
    ]
    factor = GEAR_RATIO / ENCODER_TICKS_PER_ROTATION

    def __init__(self, pins=ENCODER_PINS):
        self.hall_sensor1 = digitalio.DigitalInOut(pins[0])
        self.hall_sensor1.direction = digitalio.Direction.INPUT
        self.hall_sensor1.pull = digitalio.Pull.UP

        self.hall_sensor2 = digitalio.DigitalInOut(pins[1])
        self.hall_sensor2.direction = digitalio.Direction.INPUT
        self.hall_sensor2.pull = digitalio.Pull.UP

        self.velocity = 0
        return

    def __get_state__(self):
        state1 = self.hall_sensor1.value
        state2 = self.hall_sensor2.value
        return state1, state2

    @staticmethod
    def __ticks_to_distance(count):
        factor = 1 / ENCODER_TICKS_PER_ROTATION / 2 / GEAR_RATIO
        distance = count * factor * np.pi * WHEEL_DIAMETER
        return distance

    def read(self, distance=0):
        state0 = self.__get_state__()
        state1 = state0

        count = 0
        while count < 2800:
            while state1 == state0:
                state1 = self.__get_state__()

            idx0 = self.codes.index(state0)
            idx1 = self.codes.index(state1)

            diff = idx1 - idx0
            if diff in [1, -3]:
                count -= 1
                bad = 0
            elif diff in [-1, 3]:
                count += 1
                bad = 0
            else:
                bad = 1

            print(bad, diff, count, distance + self.__ticks_to_distance(count))
            state0 = state1

        distance = self.__ticks_to_distance(count)
        return distance


def calibrate_motor(motor):
    i = -1
    motor.throttle = i
    motor.throttle = i
    while i < 0:
        i += 0.01
        motor.throttle = i
        motor.throttle = i
        time.sleep(0.1)
        print(motor.throttle)

    while i > -0.99:
        i -= 0.01
        motor.throttle = i
        motor.throttle = i
        time.sleep(0.1)
        print(motor.throttle)

    motor.throttle = None
    motor.throttle = None
    return None


def compute_motor_commands(omega, theta, sum_torque, pid=(6.0, 0.9, 0.3)):
    torque = np.sin(theta)

    sum_torque += torque
    sum_torque = np.clip(sum_torque, -1.5, 1.5)

    proportional, integral, derivative = pid
    u = proportional * torque + integral * sum_torque + derivative * omega
    return np.clip(u, -1, 1)


def robot_tipped_over(acc_y):
    value = abs(acc_y) > 10
    return value


def main():
    imu = ImuSensor(scl=SCL0, sda=SDA0)
    battery = Battery()
    while True:
        measurements = imu.read()
        print(measurements)

        measurements = battery.get_voltage()
        print(measurements)
        time.sleep(0.01)


def initialize_motors():
    # Initialize DC motors
    m1a = pwmio.PWMOut(board.GP8, frequency=10000)
    m1b = pwmio.PWMOut(board.GP9, frequency=10000)
    motor1 = motor.DCMotor(m1a, m1b)

    m2a = pwmio.PWMOut(board.GP10, frequency=10000)
    m2b = pwmio.PWMOut(board.GP11, frequency=10000)
    motor2 = motor.DCMotor(m2a, m2b)

    motor1.decay_mode = motor.FAST_DECAY
    motor2.decay_mode = motor.FAST_DECAY
    return motor1, motor2


def initialize_buttons():
    # Initialize buttons
    btn1 = digitalio.DigitalInOut(board.GP20)
    btn2 = digitalio.DigitalInOut(board.GP21)
    btn1.direction = digitalio.Direction.INPUT
    btn2.direction = digitalio.Direction.INPUT
    btn1.pull = digitalio.Pull.UP
    btn2.pull = digitalio.Pull.UP

    return btn1, btn2


if __name__ == "__main__":
    main()
