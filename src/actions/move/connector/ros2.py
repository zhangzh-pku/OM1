import logging
import time
import pygame

from actions.base import ActionConnector
from actions.move.interface import MoveInput

import hid

class MoveRos2Connector(ActionConnector[MoveInput]):

    def __init__(self):
        # pygame.init()
        self.joysticks = []
        
        logging.info(f"Joystick")
        
        self.vendor_id=""
        self.product_id=""
        self.gamepad=None

        self.cb: list[str] = []

        for device in hid.enumerate():
            logging.info(f"device {device['product_string']}") 
            if "Xbox Wireless Controller" in device['product_string']:
                self.vendor_id  = device['vendor_id']
                self.product_id = device['product_id']
                self.gamepad = hid.Device(self.vendor_id, self.product_id)
                #self.gamepad.open(self.vendor_id, self.product_id)
                #self.gamepad.set_nonblocking(True)
                logging.info(f"Connected {device['product_string']} {self.vendor_id} {self.product_id}")
                break

        #for i in range(0, pygame.joystick.get_count()):
            # create an Joystick object in our list
        #    self.joysticks.append(pygame.joystick.Joystick(i))
            # initialize the appended joystick
        #    self.joysticks[-1].init()
            # print a statement telling what the name of the controller is
        #    logging.info(f"Joystick {joysticks[-1].get_name()}")

    async def connect(self, output_interface: MoveInput) -> None:
        # define/clarify the datatype
        new_msg = {"thought": "", "vx": 0.0, "vy": 0.0, "vyaw": 0.0}

        if len(self.cb) > 0:
            logging.info(f"WARNING - there are manual game controller commands in the queue: {self.cb}")
            return

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

    def tick(self) -> None:
        
        time.sleep(0.1)
        
        logging.info("MoveRos2Connector Tick")
        
        if self.gamepad:

            report = list(self.gamepad.read(64))
            
            if report[14] == 0:
                if len(self.cb) > 0 and self.cb[-1] is not "end": self.cb.append("end")
            elif report[14] == 1:
                if len(self.cb) == 0 or self.cb[-1] is not "game_a": self.cb.append("game_a")
            elif report[14] == 2:
                if len(self.cb) == 0 or self.cb[-1] is not "game_b": self.cb.append("game_b")
            elif report[14] == 8:
                if len(self.cb) == 0 or self.cb[-1] is not "game_x": self.cb.append("game_x")
            elif report[14] == 16:
                if len(self.cb) == 0 or self.cb[-1] is not "game_y": self.cb.append("game_y")
            
            logging.info(f"Gamepad {self.cb}")

        if len(self.cb) > 0:
            if self.cb[-1] == "game_a":
                logging.info("ROS2 unitree: stand_up")
                del self.cb[-1]
            elif self.cb[-1] == "game_b":
                logging.info("ROS2 unitree: lay_down")
                del self.cb[-1]
            elif self.cb[-1] == "game_x":
                logging.info("ROS2 unitree: say_hello")
                del self.cb[-1]
            elif self.cb[-1] == "game_y":
                logging.info("ROS2 unitree: stretch")
                    del self.cb[-1]
        

