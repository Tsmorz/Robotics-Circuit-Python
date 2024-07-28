import board
import pwmio
from adafruit_motor import servo, motor
from ulab import numpy as np
import time

from definitions import MOTOR_FREQUENCY


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


def initialize_motors():
    # Initialize DC motors
    m1a = pwmio.PWMOut(board.GP8, frequency=MOTOR_FREQUENCY)
    m1b = pwmio.PWMOut(board.GP9, frequency=MOTOR_FREQUENCY)
    motor1 = motor.DCMotor(m1a, m1b)

    m2a = pwmio.PWMOut(board.GP10, frequency=MOTOR_FREQUENCY)
    m2b = pwmio.PWMOut(board.GP11, frequency=MOTOR_FREQUENCY)
    motor2 = motor.DCMotor(m2a, m2b)

    motor1.decay_mode = motor.FAST_DECAY
    motor2.decay_mode = motor.FAST_DECAY
    return motor1, motor2
