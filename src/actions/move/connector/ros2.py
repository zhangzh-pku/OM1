import logging
import os
import time

from actions.base import ActionConnector
from actions.move.interface import MoveInput

class MoveRos2Connector(ActionConnector[MoveInput]):

    async def connect(self, output_interface: MoveInput) -> None:

        # stub to show how to do this
        if output_interface.action == "sit":
            logging.info("AI command: sit")
        elif output_interface.action == "shake paw":
            logging.info("AI command: shake paw")
        elif output_interface.action == "dance":
            logging.info("AI command: dance")
        else:
            logging.info(f"Other move type: {output_interface.action}")
            # raise ValueError(f"Unknown move type: {output_interface.action}")

    def tick(self) -> None:
        time.sleep(0.1)
        # logging.info("MoveRos2Connector Tick")