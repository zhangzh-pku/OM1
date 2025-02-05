import logging
import time
import serial

from actions.base import ActionConnector
from actions.move.interface import MoveInput


class MoveSerialConnector(ActionConnector[MoveInput]):

    def __init__(self):
        super().__init__()

        # Open the serial port
        self.ser = serial.Serial('COM1', 9600)  # Replace 'COM1' with your Arduino's port name

    async def connect(self, output_interface: MoveInput) -> None:

        new_msg = {"move": ""}

        if output_interface.action == "be still":
            new_msg["move"] = "0"
        elif output_interface.action == "small jump":
            new_msg["move"] = "1"
        elif output_interface.action == "medium jump":
            new_msg["move"] = "2"
        elif output_interface.action == "big jump":
            new_msg["move"] = "3"
        else:
            logging.info(f"Other move type: {output_interface.action}")
            # raise ValueError(f"Unknown move type: {output_interface.action}")

        if self.ser.isOpen():
            message = f"actuator:{new_msg["move"]}\r\n"
            # Convert the string to bytes using UTF-8 encoding
            byte_data = message.encode("utf-8")
            self.ser.write(byte_data)

        logging.info(f"SendToArduinoSerial: {new_msg}")

    def tick(self) -> None:
        time.sleep(0.1)
        # logging.info("Connector Tick")
