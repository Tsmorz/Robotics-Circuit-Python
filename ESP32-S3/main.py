# SPDX-FileCopyrightText: 2020 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# CircuitPython NeoPixel Color Picker Example

import send_receive
import bluetooth
import test_internet
import busio
import board

from definitions import BAUD_RATE

option = 3
if option == 0:
    bluetooth.main()
elif option == 1:
    send_receive.main()
elif option == 2:
    test_internet.main()
else:
    uart = busio.UART(board.TX, board.RX, baudrate=BAUD_RATE, timeout=0)

    message_started = False
    while True:
        byte_read = uart.read(1)  # Read one byte over UART lines
        if not byte_read:
            # Nothing read.
            continue

        if byte_read == b"<":
            # Start of message. Start accumulating bytes, but don't record the "<".
            message = []
            message_started = True
            continue

        if message_started:
            if byte_read == b">":
                # End of message. Don't record the ">".
                # Now we have a complete message. Convert it to a string, and split it up.
                message_parts = "".join(message)[2:]
                print(message_parts)
                message_started = False

            else:
                # Accumulate message byte.
                message.append(chr(byte_read[0]))
