from dataclasses import dataclass
from enum import Enum
from actions.base import Interface

class FaceAction(str, Enum):
    CRY = "cry"
    SMILE = "smile"
    FROWN = "frown"
    
@dataclass
class FaceInput:
    action: FaceAction

@dataclass
class FaceInterface(Interface[FaceInput, FaceInput]):
    """
    A facial expression to be performed by the agent.

    Effect: Performs a given facial expression.
    """

    input: FaceInput
    output: FaceInput
