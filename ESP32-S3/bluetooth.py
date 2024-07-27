# SPDX-FileCopyrightText: 2020 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# CircuitPython NeoPixel Color Picker Example

import board
import neopixel

from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.color_packet import ColorPacket
from adafruit_bluefruit_connect.button_packet import ButtonPacket

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService


def main():
    ble = BLERadio()
    uart_service = UARTService()
    advertisement = ProvideServicesAdvertisement(uart_service)

    pixels = neopixel.NeoPixel(board.NEOPIXEL, 10, brightness=0.1)

    while True:
        # Advertise when not connected.
        ble.start_advertising(advertisement)
        while not ble.connected:
            pass
        ble.stop_advertising()

        while ble.connected:
            if uart_service.in_waiting:
                packet = Packet.from_stream(uart_service)
                print(packet)
                if isinstance(packet, ButtonPacket):
                    print(packet.button)
                    string = "Helloooo"
                    uart.write(bytes(f"<B,{string}>", "ascii"))


if __name__ == "__main__":
    main()
