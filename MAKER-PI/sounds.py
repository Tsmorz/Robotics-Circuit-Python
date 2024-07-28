import simpleio

from definitions import (
    PIEZO_PIN,
    MELODY_DURATION,
    MELODY_NOTE,
)


def play_start_up_tune() -> None:
    for i in range(len(MELODY_NOTE)):
        simpleio.tone(PIEZO_PIN, MELODY_NOTE[i], duration=MELODY_DURATION[i])

    return None
