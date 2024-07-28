import board
import digitalio


def initialize_buttons():
    # Initialize buttons
    btn1 = digitalio.DigitalInOut(board.GP20)
    btn2 = digitalio.DigitalInOut(board.GP21)
    btn1.direction = digitalio.Direction.INPUT
    btn2.direction = digitalio.Direction.INPUT
    btn1.pull = digitalio.Pull.UP
    btn2.pull = digitalio.Pull.UP

    return btn1, btn2
