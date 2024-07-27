from ulab import numpy as np
from definitions import GRAVITY


def sign(number):
    if number >= 0:
        return 1
    else:
        return -1


def least_squares(A, b):
    solution = np.linalg.inv(A) @ b
    return solution


def least_norm(A, b):
    pseudo_inv = np.linalg.inv(A.T @ A) @ A.T
    solution = pseudo_inv @ b
    return solution


def rad2deg(radians):
    degrees = radians * 180 / np.pi
    return degrees


def deg2rad(degrees):
    radians = degrees * np.pi / 180
    return radians


def map_ranges(x, input_range, output_range):
    output_start, output_end = output_range
    input_start, input_end = input_range
    slope = (output_end - output_start) / (input_end - input_start)
    output = output_start + slope * (x - input_start)
    return output


def angle_from_acceleration(acceleration):
    angle = np.asin(acceleration / GRAVITY)
    return angle
