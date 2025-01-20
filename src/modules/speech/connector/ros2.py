from modules.base import ModuleConnector

from modules.speech.interface import SpeechInput


class SpeechRos2Connector(ModuleConnector[SpeechInput]):
    async def connect(self, output_interface: SpeechInput) -> None:
        pass
