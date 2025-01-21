from actions.base import ActionConnector

from actions.speech.interface import SpeechInput


class SpeechRos2Connector(ActionConnector[SpeechInput]):
    async def connect(self, output_interface: SpeechInput) -> None:
        pass
