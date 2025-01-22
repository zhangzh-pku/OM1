from actions.base import ActionImplementation

from actions.speech.interface import SpeechInput


class SpeechPassthroughImplementation(ActionImplementation[SpeechInput, SpeechInput]):
    """
    A passthrough implementation of the speech action. Output is the same as the input.
    """

    async def execute(self, input_interface: SpeechInput) -> SpeechInput:
        return input_interface
