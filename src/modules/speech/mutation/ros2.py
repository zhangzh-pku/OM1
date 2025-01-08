from modules.base import ModuleMutation

from modules.speech.interface import SpeechInput


class SpeechRos2Mutation(ModuleMutation[SpeechInput]):
    async def mutate(self, output_interface: SpeechInput) -> None:
        pass
