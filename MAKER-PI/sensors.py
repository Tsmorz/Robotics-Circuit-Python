import busio
import time
from analogio import AnalogIn
import digitalio

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


def robot_tipped_over(acc_y):
    value = abs(acc_y) > 10
    return value


def calibrate_sensors(imu):
    for i in range(10):
        measurements = imu.read()
        print(measurements)


def main():
    imu = ImuSensor(scl=SCL0, sda=SDA0)
    battery = Battery()
    while True:
        measurements = imu.read()
        print(measurements)

        measurements = battery.get_voltage()
        print(measurements)
        time.sleep(0.01)


if __name__ == "__main__":
    main()
