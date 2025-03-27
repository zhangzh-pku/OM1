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

        URID = getattr(self.config, "URID", None)

        if URID is None:
            # Log the domain id being used
            logging.warning(f"Aborting Move, no URID provided: {URID}")
            return
        else:
            logging.warning(f"Move system is using URID:{URID}")

        self.cmd_vel = f"{URID}/c3/cmd_vel"
        try:
            self.session = zenoh.open(zenoh.Config())
            logging.info("Zenoh client opened")
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

        logging.info(f"SendThisToZenoh: {output_interface.action}")

        if output_interface.action == "turn left":
            logging.info(f"Zenoh command: {output_interface.action}")
            self.pub_twist(0.0, 0.2)
        elif output_interface.action == "turn right":
            logging.info(f"Zenoh command: {output_interface.action}")
            self.pub_twist(0.0, -0.2)
        elif output_interface.action == "move forwards":
            logging.info(f"Zenoh command: {output_interface.action}")
            self.pub_twist(0.5, 0.0)
        elif output_interface.action == "move back":
            logging.info(f"Zenoh command: {output_interface.action}")
            self.pub_twist(-0.5, 0.0)
        elif output_interface.action == "avoid left":
            logging.info(f"Zenoh command: {output_interface.action}")
            self.pub_twist(1.0, 0.5)
        elif output_interface.action == "avoid right":
            logging.info(f"Zenoh command: {output_interface.action}")
            self.pub_twist(1.0, -0.5)
        elif output_interface.action == "move back":
            logging.info(f"Zenoh command: {output_interface.action}")
            self.pub_twist(-0.1, 0.0)
        elif output_interface.action == "stand still":
            logging.info(f"Zenoh command: {output_interface.action}")
            # do nothing
        else:
            logging.info(f"Unknown move type: {output_interface.action}")

    def tick(self) -> None:

        time.sleep(0.1)
