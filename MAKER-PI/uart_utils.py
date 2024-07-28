import busio
from definitions import TX0, RX0, BAUD_RATE

STARTING_CHAR = b"<"
ENDING_CHAR = b">"


def initialize_uart():
    uart = busio.UART(TX0, RX0, baudrate=BAUD_RATE, timeout=0)
    return uart


def write_to_serial(uart, message, encoding="ascii") -> None:
    uart.write(bytes(f"<B,{message}>", encoding))
    return None


def read_from_serial(uart):
    message_started = False

    while True:
        byte_read = uart.read(1)  # Read one byte over UART lines
        if not byte_read:
            # Nothing read.
            continue

        if byte_read == STARTING_CHAR:
            # Start of message. Start accumulating bytes, but don't record the "<".
            message = []
            message_started = True
            continue

        if message_started:
            if byte_read == ENDING_CHAR:
                # End of message. Don't record the ">".
                # Now we have a complete message. Convert it to a string, and split it up.
                message_parts = "".join(message)[2:]
                break

            else:
                # Accumulate message byte.
                message.append(chr(byte_read[0]))

    message = "".join(message)[2:]
    return message.split(", ")
