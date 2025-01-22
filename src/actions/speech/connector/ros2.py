import logging
from actions.base import ActionConnector
from actions.speech.interface import SpeechInput


class SpeechRos2Connector(ActionConnector[SpeechInput]):
    async def connect(self, output_interface: SpeechInput) -> None:
        sentence_to_speak = {'sentence': output_interface.sentence}
        logging.info(f"SendThisToROS2: {sentence_to_speak}")
