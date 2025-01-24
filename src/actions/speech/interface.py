from dataclasses import dataclass

from actions.base import Interface


@dataclass
class SpeechInput:
    sentence: str


@dataclass
class Speech(Interface[SpeechInput, SpeechInput]):
    """
    Words to be spoken by the agent.

    Effect: Allows the agent to speak.
    """

    input: SpeechInput
    output: SpeechInput
