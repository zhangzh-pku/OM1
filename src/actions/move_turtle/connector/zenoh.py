import logging
import time

import zenoh
from pycdr import cdr
from pycdr.types import float64

from actions.base import ActionConfig, ActionConnector
from actions.move_safe.interface import MoveInput


# Declare the types of Twist message to be encoded and published via zenoh
@cdr
class Vector3:
    x: float64
    y: float64
    z: float64


@cdr
class Twist:
    linear: Vector3
    angular: Vector3


class MoveZenohConnector(ActionConnector[MoveInput]):

    def __init__(self, config: ActionConfig):
        super().__init__(config)
        self.session = None
        try:
            # Initiate the zenoh-net API
            self.session = zenoh.open()
            logging.info("Zenoh Opened")
        except Exception as e:
            logging.error(f"Error opening Zenoh client: {e}")

    async def connect(self, output_interface: MoveInput) -> None:

        if output_interface.action == "turn left":
            logging.info("Zenoh command: turn left")
            t = Twist(
                linear=Vector3(x=0.0, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=0.2),
            ).serialize()
            self.session.put("cmd_vel", t)
        elif output_interface.action == "turn right":
            logging.info("Zenoh command: turn right")
            t = Twist(
                linear=Vector3(x=0.0, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=-0.2),
            ).serialize()
            self.session.put("cmd_vel", t)
        elif output_interface.action == "move forwards":
            logging.info("Zenoh command: move forwards")
            t = Twist(
                linear=Vector3(x=0.5, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=0.0),
            ).serialize()
            self.session.put("cmd_vel", t)
        elif output_interface.action == "move back":
            logging.info("Zenoh command: move back")
            t = Twist(
                linear=Vector3(x=-0.5, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=0.0),
            ).serialize()
            self.session.put("cmd_vel", t)
        elif output_interface.action == "stand still":
            logging.info("Zenoh command: stand still")
            # do nothing
        else:
            logging.info(f"Unknown move type: {output_interface.action}")

        logging.info(f"SendThisToZenoh: {output_interface.action}")

    def tick(self) -> None:

        time.sleep(0.1)
