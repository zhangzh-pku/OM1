import asyncio
import logging
import time
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import zenoh
from numpy import array
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks

from inputs.base import SensorConfig
from inputs.base.loop import FuserInput
from providers import BatteryStatus, IOProvider, StatusProvider, TeleopsStatus
from zenoh_idl import sensor_msgs


@dataclass
class Message:
    timestamp: float
    message: str


class TurtleBot4BattLIDARBump(FuserInput[str]):
    """
    TurtleBot4 LIDAR, Battery, and bump inputs.

    Takes specific TurtleBot4 ROS2/Zenoh messages, converts them to
    text strings, and sends them to the fuser.

    Maintains a buffer of processed messages.
    """

    def __init__(self, config: SensorConfig = SensorConfig()):
        super().__init__(config)

        api_key = getattr(self.config, "api_key", None)

        # IO provider
        self.io_provider = IOProvider()

        # Staus provider
        self.status_provider = StatusProvider(api_key=api_key)

        # Buffer for storing the final output
        self.messages: list[Message] = []

        # Status variables
        self.battery_status = None
        self.lidar_status = None
        self.hazard_status = None

        # Battery status variables
        self.battery_percent = 0.0
        self.battery_voltage = 0.0
        self.battery_temperature = 0
        self.battery_timestamp = time.time()
        self.is_docked = False

        # LIDAR processing vector
        self.vectorM2 = np.zeros(1080)
        self.vectorM1 = np.zeros(1080)

        # Intensity threshold for LIDAR (values between 0 and 47)
        self.intensity_threshold = 1

        # Start Zenoh session
        logging.info("Opening Zenoh TurtleBot4 session...")
        conf = zenoh.Config()
        self.z = zenoh.open(conf)

        logging.info(f"Config: {self.config}")

        self.URID = getattr(self.config, "URID", "default")
        logging.info(f"Using TurtleBot4 URID: {self.URID}")

        # Zenoh subscribers
        logging.info("Creating Zenoh TurtleBot4 Subscribers")
        self.scans = self.z.declare_subscriber(
            f"{self.URID}/pi/scan", self.listener_scan
        )
        self.batts = self.z.declare_subscriber(
            f"{self.URID}/c3/battery_state", self.listener_battery
        )
        self.bump = self.z.declare_subscriber(
            f"{self.URID}/c3/hazard_detection", self.listener_hazard
        )
        self.dock = self.z.declare_subscriber(
            f"{self.URID}/c3/dock_status", self.listener_dock
        )

        self.descriptor_for_LLM = "Body State"

    def listener_battery(self, sample):
        """
        Process battery status updates
        """
        battery = sensor_msgs.BatteryState.deserialize(sample.payload.to_bytes())
        self.battery_percent = int(battery.percentage * 100)
        self.battery_voltage = battery.voltage
        self.battery_temperature = round(battery.temperature, 2)
        self.battery_timestamp = battery.header.stamp.sec

        if self.battery_percent < 5:
            self.battery_status = "CRITICAL: your battery is almost empty. Immediately move to your charging station and recharge."
        elif self.battery_percent < 15:
            self.battery_status = "Caution: your battery is running low. Consider finding your charging station and recharging."
        else:
            self.battery_status = None

    def listener_hazard(self, sample):
        """Process hazard detection updates"""
        hazard = sensor_msgs.HazardDetectionVector.deserialize(
            sample.payload.to_bytes()
        )

        if hazard.detections and len(hazard.detections) > 0:
            for haz in hazard.detections:
                if haz.type == 1:
                    if haz.header.frame_id == "bump_front_right":
                        self.hazard_status = (
                            "DANGER: you are hitting something on your front right."
                        )
                        # if this hazard exists, report it immediately, and don't overwrite
                        # with less important information
                        return
                    if haz.header.frame_id == "bump_front_left":
                        self.hazard_status = (
                            "DANGER: you are hitting something on your front left."
                        )
                        return
                    if haz.header.frame_id == "bump_front_center":
                        self.hazard_status = (
                            "DANGER: you are hitting something right in front of you."
                        )
                        return

    def listener_scan(self, sample):
        """Process LIDAR scan updates"""
        scan = sensor_msgs.LaserScan.deserialize(sample.payload.to_bytes())
        clusters = []

        # smoothing
        # Add the new data, with the last two data vectors
        # then divide by 3 -> simple averaging
        smv = np.add(scan.ranges, self.vectorM1)
        smv = np.add(smv, self.vectorM2)
        smv = np.divide(smv, 3)
        self.vectorM2 = self.vectorM1
        self.vectorM1 = scan.ranges

        for distance, intensity in list(zip(smv, scan.intensities)):
            # let's look only at close things (< 0.50m)
            if distance <= 0.50:
                if intensity >= self.intensity_threshold:
                    # convert to cm and flip to emphasize near returns
                    clusters.append(51 - int(distance * 100))
                else:
                    clusters.append(0)
            else:
                clusters.append(0)

        a = array(clusters)
        # roll the data so the "nose" of the robot is in the middle (around 530)
        a = np.roll(a, 278)
        # flip so we are in the human/robot frame
        a = a[::-1]
        # smooth a bit
        x_kde = gaussian_filter1d(a, 5)
        x_kde = x_kde[200:880]  # chop off data from the back of the robot
        peaks, _ = find_peaks(x_kde, height=0)

        max_x_peak = -10  # arbitrary negative number
        max_y_peak = -10
        for x in peaks:
            y = x_kde[int(x)]
            if y > max_y_peak:
                max_y_peak = y
                max_x_peak = int(x)

        # the array has length 680
        proximity = "close to"
        direction = "on your left"
        if max_x_peak > 0:
            if max_y_peak > 20:
                proximity = "hitting"
            if max_x_peak > 453:
                direction = "on your right"
            elif max_x_peak > 227:
                direction = "in front of you"
            self.lidar_status = f"CAUTION: You are {proximity} something {direction}."
        else:
            self.lidar_status = None

    def listener_dock(self, sample):
        """
        Process docking status updates
        """
        dock = sensor_msgs.DockStatus.deserialize(sample.payload.to_bytes())
        self.is_docked = dock.is_docked

    async def update_status(self):
        """
        Report the battery status to the status provider.
        """
        self.status_provider.share_status(
            TeleopsStatus(
                machine_name="TurtleBot4",
                update_time=time.time(),
                battery_status=BatteryStatus(
                    battery_level=self.battery_percent,
                    temperature=self.battery_temperature,
                    voltage=self.battery_voltage,
                    timestamp=self.battery_timestamp,
                    charging_status=self.is_docked,
                ),
            )
        )

    async def _poll(self) -> List[str]:
        """
        Poll for new state data.

        Returns
        -------
        List[str]
        """

        # Does the complexity of this seem confusing and kinda pointless to you?
        # It's on our radar and your patience is appreciated
        await asyncio.sleep(2.0)

        await self.update_status()

        logging.info(
            f"TB4 batt:{self.battery_status} lidar:{self.lidar_status} hazard:{self.hazard_status}"
        )

        return [self.battery_status, self.lidar_status, self.hazard_status]

    async def _raw_to_text(self, raw_input: List[str]) -> Optional[Message]:
        """
        Process raw lowstate to generate text description.

        Parameters
        ----------
        raw_input : List[str]
            State data to be processed

        Returns
        -------
        Message
            Timestamped message containing description
        """
        battery = raw_input[0]
        lidar = raw_input[1]
        hazard = raw_input[2]

        if hazard:
            message = hazard
            return Message(timestamp=time.time(), message=message)
        # when there is a hazard, that ALWAYS takes precendence
        # so we want the above to return
        # and we do not care about lidar etc
        if lidar:
            message = lidar
            return Message(timestamp=time.time(), message=message)

        if battery:
            message = battery
            return Message(timestamp=time.time(), message=message)

    async def raw_to_text(self, raw_input: List[float]):
        """
        Convert raw lowstate to text and update message buffer.

        Parameters
        ----------
        raw_input : List[float]
            Raw lowstate data to be processed
        """
        pending_message = await self._raw_to_text(raw_input)

        if pending_message is not None:
            self.messages.append(pending_message)

    def formatted_latest_buffer(self) -> Optional[str]:
        """
        Format and clear the latest buffer contents.

        Formats the most recent message with timestamp and class name,
        adds it to the IO provider, then clears the buffer.

        Returns
        -------
        Optional[str]
            Formatted string of buffer contents or None if buffer is empty
        """
        if len(self.messages) == 0:
            return None

        latest_message = self.messages[-1]

        result = f"""
{self.descriptor_for_LLM} INPUT
// START
{latest_message.message}
// END
"""

        self.messages = []
        self.hazard_status = None

        return result
