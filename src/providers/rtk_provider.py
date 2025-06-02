import logging
import threading
import time
from typing import Optional

import serial

from .singleton import singleton


@singleton
class RtkProvider:
    """
    RTK Provider.

    This class implements a singleton pattern to manage:
        * RTK data from serial

    Parameters
    ----------
    serial_port: str = ""
    """

    def __init__(self, serial_port: str = ""):
        """
        Robot and sensor configuration
        """

        logging.info("Booting RTK Provider")

        baudrate = 115200
        timeout = 1

        self.serial_connection = None
        try:
            self.serial_connection = serial.Serial(
                serial_port, baudrate, timeout=timeout
            )
            logging.info(f"Connected to {serial_port} at {baudrate} baud")
        except serial.SerialException as e:
            logging.error(f"Error: {e}")

        self._rtk: Optional[dict] = None

        self.lat = ""
        self.lon = ""
        self.alt = 0.0
        self.sats = 0
        self.time_utc = ""

        self.running = False
        self._thread: Optional[threading.Thread] = None
        self.start()

    def magGPSProcessor(self, data):
        # Used whenever there is a connected
        # nav Arduino on serial
        # Parses lines like:
        # - GPS:37.7749N,-122.4194W,KN:0.12,HEAD:84.1,ALT:30.5,SAT:7,TIME:3:14:24:546
        # - YPR: 134.57, -3.20, 1.02
        # - HDG (DEG): 225.0 SW NTC_HDG: 221.3
        try:
            # if data.startswith("HDG (DEG):"):
            #     parts = data.split()
            #     if len(parts) >= 4:
            #         # that's a HDG packet
            #         self.yaw_mag_0_360 = float(parts[2])
            #         self.yaw_mag_cardinal = parts[3]
            #         logging.debug(f"MAG: {self.yaw_mag_0_360}")
            #     else:
            #         logging.warning(f"Unable to parse heading: {data}")
            # elif data.startswith("YPR:"):
            #     yaw, pitch, roll = map(str.strip, data[4:].split(","))
            #     logging.debug(
            #         f"Orientation is Yaw: {yaw}°, Pitch: {pitch}°, Roll: {roll}°."
            #     )
            # el
            logging.info({data})

            if data.startswith("GPS:"):
                try:
                    parts = data[4:].split(",")
                    lat = parts[0]
                    lon = parts[1]
                    heading = parts[3].split(":")[1]
                    alt = parts[4].split(":")[1]
                    sats = parts[5].split(":")[1]
                    self.lat = lat
                    self.lon = lon
                    self.alt = float(alt)
                    self.sats = int(sats)
                    if len(parts) >= 6:
                        time = parts[6][5:]
                        self.time_utc = time
                    logging.debug(
                        (
                            f"Current location is {lat}, {lon} at {alt}m altitude. "
                            f"GPS Heading {heading}° with {sats} satellites locked. "
                            f"The time, if available, is {self.time_utc}."
                        )
                    )
                except Exception as e:
                    logging.warning(f"Failed to parse RTK: {data} ({e})")
        except Exception as e:
            logging.warning(f"Error processing serial RTK input: {data} ({e})")

        self._rtk = {
            "gps_lat": self.lat,
            "gps_lon": self.lon,
            "gps_alt": self.alt,
            "gps_sats": self.sats,
            "gps_time_utc": self.time_utc,
        }

    def start(self):
        """
        Starts the RTK Provider and processing thread
        if not already running.
        """
        if self._thread and self._thread.is_alive():
            return

        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        """
        Main loop for the RTK provider.
        """
        while self.running:

            if self.serial_connection:
                # Read a line, decode, and remove whitespace
                data = self.serial_connection.readline().decode("utf-8").strip()
                logging.debug(f"Serial RTK: {data}")
                self.magRTKProcessor(data)

            time.sleep(0.05)

    def stop(self):
        """
        Stop the RTK provider.
        """
        self.running = False
        if self._thread:
            logging.info("Stopping RTK provider")
            self._thread.join(timeout=5)

    @property
    def data(self) -> Optional[dict]:
        # """
        # Get the current robot rtk data
        # """
        return self._rtk
