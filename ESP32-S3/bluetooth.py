# SPDX-FileCopyrightText: 2020 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# CircuitPython NeoPixel Color Picker Example

import board
import busio

from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

from definitions import BAUD_RATE


def main():
    ble = BLERadio()
    uart_service = UARTService()
    advertisement = ProvideServicesAdvertisement(uart_service)
    uart = busio.UART(board.TX, board.RX, baudrate=BAUD_RATE, timeout=0)

    while True:
        # Advertise when not connected.
        ble.start_advertising(advertisement)
        while not ble.connected:
            pass
        ble.stop_advertising()

        while ble.connected:
            if uart_service.in_waiting:
                packet = Packet.from_stream(uart_service)

                if isinstance(packet, ButtonPacket):
                    btn = int(packet.button)
                    print(btn)

                    if up_arrow_pressed(btn):
                        m1 = -1
                        m2 = -1
                    elif down_arrow_pressed(btn):
                        m1 = 1
                        m2 = 1
                    elif left_arrow_pressed(btn):
                        m1 = -1
                        m2 = 1
                    elif right_arrow_pressed(btn):
                        m1 = 1
                        m2 = -1
                    elif stop_pressed(btn):
                        m1 = 0
                        m2 = 0
                    else:
                        continue

                    string = f"{m1}, {m2}"
                    uart.write(bytes(f"<B,{string}>", "ascii"))
                    print(m1, m2)


def up_arrow_pressed(btn):
    return btn == 5


def down_arrow_pressed(btn):
    return btn == 6


def left_arrow_pressed(btn):
    return btn == 7


def right_arrow_pressed(btn):
    return btn == 8


def stop_pressed(btn):
    return btn == 1


if __name__ == "__main__":
    main()
