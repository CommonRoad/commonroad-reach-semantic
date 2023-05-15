import enum
from enum import Enum, auto


@enum.unique
class OutgoingDirection(Enum):
    LEFT = auto()
    STRAIGHT = auto()
    RIGHT = auto()

    def __str__(self):
        return self.name.lower()
