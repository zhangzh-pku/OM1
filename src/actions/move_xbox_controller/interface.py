from dataclasses import dataclass

from actions.base import Interface


@dataclass
class IDLEInput:
    action: str


@dataclass
class XboxController(Interface[IDLEInput, IDLEInput]):
    """
    xbox controller interface
    """

    input: IDLEInput
    output: IDLEInput
