from dataclasses import dataclass
from enum import Enum
from modules.base import Interface

class MovementAction(str, Enum):
    STAND_UP = "stand up"
    LAY_DOWN = "lay down"
    POUNCE = "pounce"
    STAND_STILL = "stand still"
    SIT = "sit"
    STRETCH = "stretch"
    DANCE = "dance"
    SHAKE_PAW = "shake paw"
    WALK = "walk"
    RUN = "run"
    JUMP = "jump"
    MOVE_BACK = "move back"
    TURN_LEFT = "turn left"
    TURN_RIGHT = "turn right"

@dataclass
class MoveInput:
    action: MovementAction

@dataclass
class MoveInterface(Interface[MoveInput, MoveInput]):
    """
    A movement to be performed by the agent.

    Effect: Allows the agent to move.
    """

    input: MoveInput
    output: MoveInput
