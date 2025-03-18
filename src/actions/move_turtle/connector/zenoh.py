import logging
import time
from dataclasses import dataclass

import zenoh
from pycdr2 import IdlStruct
from pycdr2.types import float64

from actions.base import ActionConfig, ActionConnector
from actions.move_turtle.interface import MoveInput


@dataclass
class Vector3(IdlStruct, typename="Vector3"):
    x: float64
    y: float64
    z: float64


@dataclass
class Twist(IdlStruct, typename="Twist"):
    linear: Vector3
    angular: Vector3


class MoveZenohConnector(ActionConnector[MoveInput]):

    def __init__(self, config: ActionConfig):
        super().__init__(config)
        self.session = None
        self.cmd_vel = "cmd_vel"
        try:
            # Initiate the zenoh-net API
            self.session = zenoh.open(zenoh.Config())
            logging.info("Zenoh Opened")
        except Exception as e:
            logging.error(f"Error opening Zenoh client: {e}")

    def pub_twist(self, linear, angular):
        if self.session is None:
            logging.info("No open Zenoh session, returning")
            return
        logging.info("Pub twist: {} - {}".format(linear, angular))
        t = Twist(
            linear=Vector3(x=float(linear), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(angular)),
        )
        self.session.put(self.cmd_vel, t.serialize())

    async def connect(self, output_interface: MoveInput) -> None:

        if output_interface.action == "turn left":
            logging.info("Zenoh command: turn left")
            self.pub_twist(0.0, 0.2)
        elif output_interface.action == "turn right":
            logging.info("Zenoh command: turn right")
            self.pub_twist(0.0, -0.2)
        elif output_interface.action == "move forwards":
            logging.info("Zenoh command: move forwards")
            self.pub_twist(0.5, 0.0)
        elif output_interface.action == "move back":
            logging.info("Zenoh command: move back")
            self.pub_twist(-0.5, 0.0)
        elif output_interface.action == "stand still":
            logging.info("Zenoh command: stand still")
            # do nothing
        else:
            logging.info(f"Unknown move type: {output_interface.action}")

        logging.info(f"SendThisToZenoh: {output_interface.action}")

    def tick(self) -> None:

        time.sleep(0.1)
