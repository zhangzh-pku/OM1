import logging

from modules.base import ModuleConnector
from modules.move.interface import MoveInput

class MoveRos2Connector(ModuleConnector[MoveInput]):
    async def connect(self, output_interface: MoveInput) -> None:

        # define/clarify the datatype
        new_msg = {"thought": "", "vx": 0.0, "vy": 0.0, "vyaw": 0.0}

        if output_interface.action == "stand up":
            new_msg["thought"] = "stand_up"
        elif output_interface.action == "lay down":
            new_msg["thought"] = "lay_down"
        elif output_interface.action == "pounce":
            new_msg["thought"] = "pounce"
        elif output_interface.action == "stand still":
            new_msg["thought"] = "stand_still"
        elif output_interface.action == "sit":
            new_msg["thought"] = "sit"
        elif output_interface.action == "stretch":
            new_msg["thought"] = "stretch"
        elif output_interface.action == "dance":
            new_msg["thought"] = "dance"
        elif output_interface.action == "shake paw":
            new_msg["thought"] = "say_hello"
        elif output_interface.action == "walk":  # CCW(LEFT) + ; CW(RIGHT) -
            new_msg["thought"] = "move"
            new_msg["vx"] = 0.4
        elif output_interface.action == "run":
            new_msg["thought"] = "move"
            new_msg["vx"] = 0.8
        elif output_interface.action == "jump":
            new_msg["thought"] = "move"
            new_msg["vx"] = 1.2
        elif output_interface.action == "move back":
            new_msg["thought"] = "move"
            new_msg["vx"] = -0.4
        elif output_interface.action == "turn left":
            new_msg["thought"] = "move"
            new_msg["vyaw"] = 0.4
        elif output_interface.action == "turn right":
            new_msg["thought"] = "move"
            new_msg["vyaw"] = -0.4
        else:
            logging.info(f"Unknown move type: {output_interface.action}")
            # raise ValueError(f"Unknown move type: {output_interface.action}")

        logging.info(f"SendThisToROS2: {new_msg}")
