from modules.base import ModuleImpl

from modules.speech.interface import SpeechInput


class SpeechPassthroughImpl(ModuleImpl[SpeechInput, SpeechInput]):
    """
    A passthrough implementation of the speech module. Output is the same as the input.
    """

    async def execute(self, input_interface: SpeechInput) -> SpeechInput:
        return input_interface
