from dataclasses import dataclass
from modules.base import Interface


@dataclass
class SpeechInput:
    sentence: str


@dataclass
class Speech(Interface[SpeechInput, SpeechInput]):
    """
    A speech action to be performed by the embodied agent.

    Effect: Allows the agent to speak.
    """

    input: SpeechInput
    output: SpeechInput
