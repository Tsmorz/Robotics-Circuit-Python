import board

SDA0 = board.GP16
SCL0 = board.GP17

SDA1 = board.GP26
SCL1 = board.GP27

TX0 = board.GP0
RX0 = board.GP1

# Battery definitions
BATTERY = board.A3
BATTERY_VOLTAGE = 6  # volts

# Wheel Encoder Definitions
ENCODER_TICKS_PER_ROTATION = 14
ENCODER_PINS = (board.GP2, board.GP3)
ENCODER1a_PIN = board.GP2
ENCODER1b_PIN = board.GP3
ENCODER2a_PIN = board.GP4
ENCODER2b_PIN = board.GP5
GEAR_RATIO = 100

WHEEL_DIAMETER = 0.065  # meters

MOTOR_FREQUENCY = 10000

GRAVITY = 9.81

# Melody
MELODY_NOTE = [659, 659, 0, 659, 0, 523, 659, 0, 784]
MELODY_DURATION = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.2]

# Define pin connected to piezo buzzer
PIEZO_PIN = board.GP22

NEOPIXEL_PIN = board.GP18

BAUD_RATES = [
    1200,
    2400,
    4800,
    9600,
    14400,
    19200,
    28800,
    38400,
    57600,
    76800,
    115200,
    230400,
    250000,
    460800,
    921600,
    1000000,
]
BAUD_RATE = BAUD_RATES[6]
