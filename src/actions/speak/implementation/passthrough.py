from actions.base import ActionImplementation
from actions.speak.interface import SpeakInput


class SpeakPassthroughImplementation(ActionImplementation[SpeakInput, SpeakInput]):
    """
    A passthrough implementation of the speak action. Output is the same as the input.
    """

    async def execute(self, input_interface: SpeakInput) -> SpeakInput:
        return input_interface
