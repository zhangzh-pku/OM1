import logging
import os
import time

import hid

from actions.base import ActionConnector
from actions.move.interface import MoveInput
from unitree.unitree_sdk2py.go2.sport.sport_client import SportClient


class MoveRos2Connector(ActionConnector[MoveInput]):

    def __init__(self):
        # pygame.init()
        self.joysticks = []

        self.vendor_id = ""
        self.product_id = ""
        self.button_previous = None
        self.gamepad = None

        self.cb: list[str] = []

        for device in hid.enumerate():
            logging.debug(f"device {device['product_string']}")
            if "Xbox Wireless Controller" in device["product_string"]:
                self.vendor_id = device["vendor_id"]
                self.product_id = device["product_id"]
                self.gamepad = hid.Device(self.vendor_id, self.product_id)
                logging.info(
                    f"Connected {device['product_string']} {self.vendor_id} {self.product_id}"
                )
                break

        # create sport client
        self.sport_client = None

        self.UNIEN0 = os.getenv("UNITREE_WIRED_ETHERNET")
        if self.UNIEN0 is not None and self.UNIEN0 != "SIM":
            # Set up Unitree subscriber unless adapater is set to "SIM""
            # ChannelFactoryInitialize(0, self.UNITREE_WIRED_ETHERNET)
            # this can only be done once, at top level
            logging.info(
                f"Move system using {self.UNIEN0} as the network Ethernet adapter"
            )
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()

    async def connect(self, output_interface: MoveInput) -> None:

        # define/clarify the datatype
        new_msg = {"thought": "", "vx": 0.0, "vy": 0.0, "vyaw": 0.0}

        if len(self.cb) > 0:
            logging.info(
                f"WARNING - there are manual game controller commands in the queue: {self.cb}"
            )
            return

        if output_interface.action == "stand up":
            logging.info("Unitree AI command: stand up")
            if self.sport_client:
                self.sport_client.StandUp()
        elif output_interface.action == "sit":
            logging.info("Unitree AI command: lay down")
            if self.sport_client:
                self.sport_client.StandDown()
        elif output_interface.action == "pounce":
            new_msg["thought"] = "pounce"
        elif output_interface.action == "stand still":
            new_msg["thought"] = "stand_still"
        elif output_interface.action == "stretch":
            logging.info("Unitree AI command: stretch")
            if self.sport_client:
                self.sport_client.Stretch()
        elif output_interface.action == "dance":
            new_msg["thought"] = "dance"
        elif output_interface.action == "shake paw":
            logging.info("Unitree AI command: shake paw")
            if self.sport_client:
                self.sport_client.Hello()
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

    def tick(self) -> None:

        time.sleep(0.1)

        # logging.info("MoveRos2Connector Tick")

        if self.gamepad:

            button_now = list(self.gamepad.read(64))[14]

            if self.button_previous == 0 and button_now > 0:
                # YAY - user just pressed a button
                # we need this logic because when the user presses a button
                # the gamepad sends a 'press' indication over and over again
                # for several hundred ms, which would create numerous
                # duplicated movement commands with a single button press
                # to prevent this, we only act when the button state changes from
                # 0 to > 0
                if button_now == 1:
                    if len(self.cb) == 0 or self.cb[-1] != "game_a":
                        self.cb.append("game_a")
                elif button_now == 2:
                    if len(self.cb) == 0 or self.cb[-1] != "game_b":
                        self.cb.append("game_b")
                elif button_now == 8:
                    if len(self.cb) == 0 or self.cb[-1] != "game_x":
                        self.cb.append("game_x")
                elif button_now == 16:
                    if len(self.cb) == 0 or self.cb[-1] != "game_y":
                        self.cb.append("game_y")
                logging.info(f"Gamepad button depressed edge {self.cb}")

            self.button_previous = button_now

        if len(self.cb) > 0:

            if self.cb[-1] == "game_a":
                logging.info("ROS2 unitree: stand_up")
                if self.sport_client:
                    self.sport_client.StandUp()
                del self.cb[-1]
            elif self.cb[-1] == "game_b":
                if self.sport_client:
                    self.sport_client.StandDown()
                logging.info("ROS2 unitree: lay_down")
                del self.cb[-1]
            elif self.cb[-1] == "game_x":
                if self.sport_client:
                    self.sport_client.Hello()
                logging.info("ROS2 unitree: say_hello")
                del self.cb[-1]
            elif self.cb[-1] == "game_y":
                if self.sport_client:
                    self.sport_client.Stretch()
                logging.info("ROS2 unitree: stretch")
                del self.cb[-1]
