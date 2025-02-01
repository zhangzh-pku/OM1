import logging

from actions.base import ActionConnector
from actions.speak.interface import SpeakInput


class SpeakRos2Connector(ActionConnector[SpeakInput]):
    async def connect(self, output_interface: SpeakInput) -> None:
        sentence_to_speak = {"sentence": output_interface.sentence}
        logging.info(f"SendThisToROS2: {sentence_to_speak}")
