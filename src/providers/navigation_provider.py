import logging
import math
import threading
import time
from typing import Optional

import bezier
import numpy as np
import zenoh
from numpy.typing import NDArray

from zenoh_idl import sensor_msgs

# from .rplidar_driver import RPDriver
from .singleton import singleton

from unitree.unitree_sdk2py.core.channel import ChannelSubscriber
from unitree.unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_
from unitree.unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

rad_to_deg = 57.2958

# gNav = None


# # def listenerNav(sample):
# #     global gScan
# #     gScan = sensor_msgs.LaserScan.deserialize(sample.payload.to_bytes())
# #     logging.debug(f"Zenoh Laserscan data: {gScan}")


# def listenerNav(sample):
#     global gScan
#     gScan = sensor_msgs.LaserScan.deserialize(sample.payload.to_bytes())
#     logging.debug(f"Zenoh Laserscan data: {gScan}")


@singleton
class NavigationProvider:
    """
    Navigation Provider.

    # This class implements a singleton pattern to manage RPLidar data streaming.

    # Parameters
    # ----------
    # wait: bool = False
    #     Whether to wait for another class to init this driver, somewhere else in the codebase
    # serial_port: str = "/dev/cu.usbserial-0001"
    #     The name of the serial port in use by the RPLidar sensor.
    # half_width_robot: float = 0.20
    #     The half width of the robot in m
    # angles_blanked: list = []
    #     Regions of the scan to disregard, runs from -180 to +180 deg
    # max_relevant_distance: float = 1.1
    #     Only consider barriers within this range, in m
    # sensor_mounting_angle: float = 180.0
    #     The angle of the sensor zero relative to the way in which it's mounted
    """

    def __init__(
        self,
        wait: bool = False,
        # serial_port: str = "/dev/cu.usbserial-0001",
        # half_width_robot: float = 0.20,
        # angles_blanked: list = [],
        # max_relevant_distance: float = 1.1,
        # sensor_mounting_angle: float = 180.0,
        URID: str = "",
        use_zenoh: bool = False,
    ):
        """
        Robot and sensor configuration
        """

        logging.info("Booting Navigation and State Provider")

        if wait:
            # no need to reinit driver
            return

        self.use_zenoh = use_zenoh

        # create subscriber
        self.low_state = None
        self.lowstate_subscriber = None

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        self.pose_subscriber = ChannelSubscriber("rt/utlidar/robot_pose", PoseStamped_)
        self.pose_subscriber.Init(self.PoseMessageHandler, 10)

        # battery state
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0
        self.battery_amperes = 0.0
        self.battery_temperature = 0

        self.body_height_cm = 0
        self.body_attitude_previous = None

        self._position = []

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw_now = 0.0

        self.running = False
        self._thread: Optional[threading.Thread] = None
        self.start()


        # self.serial_port = serial_port
        # self.half_width_robot = half_width_robot
        # self.angles_blanked = angles_blanked
        # self.max_relevant_distance = max_relevant_distance
        # self.sensor_mounting_angle = sensor_mounting_angle
        # self.URID = URID


        # self.running: bool = False
        # self.lidar = None
        # self.zen = None
        # self.scans = None

        # self._raw_scan: Optional[NDArray] = None
        # self._valid_paths: Optional[list] = None
        # self._lidar_string: str = None

        # self.paths = []
        # self.pp = []
        # self.s_vals = np.linspace(0.0, 1.0, 10)

        # for curve in self.curves:
        #     cp = curve.evaluate_multi(self.s_vals)
        #     self.paths.append(cp)
        #     pairs = list(zip(cp[0], cp[1]))
        #     self.pp.append(pairs)

        # # logging.info(self.paths)
        # # logging.info(self.pp)

        # if not self.use_zenoh:
        #     try:
        #         self.lidar = RPDriver(self.serial_port)

        #         info = self.lidar.get_info()
        #         ret = f"RPLidar Info: {info}"

        #         logging.info(ret)

        #         health = self.lidar.get_health()
        #         ret = f"RPLidar Health: {health[0]}"
        #         logging.info(ret)

        #         if health[0] == "Good":
        #             logging.info(ret)
        #         else:
        #             logging.info("there is a problem with the LIDARret")

        #         # reset to clear buffers
        #         self.lidar.reset()

        #         time.sleep(0.5)

        #     except Exception as e:
        #         logging.error(f"Error in RPLidar provider: {e}")

        # elif self.use_zenoh:
        #     try:
        #         self.zen = zenoh.open(zenoh.Config())
        #         logging.info(f"Zenoh move client opened {self.zen}")
        #         logging.info(
        #             f"TurtleBot4 move listeners starting with URID: {self.URID}"
        #         )
        #         self.zen.declare_subscriber(f"{self.URID}/pi/scan", listenerScan)
        #     except Exception as e:
        #         logging.error(f"Error opening Zenoh client: {e}")

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg

        self.battery_percentage = float(msg.bms_state.soc)
        self.battery_voltage = float(msg.power_v)
        self.battery_amperes = float(msg.power_a)
        self.battery_temperature = int(
            (msg.temperature_ntc1 + msg.temperature_ntc2) / 2
        )

        # other things you can read
        # print("FR_0 motor state: ", msg.motor_state[go2.LegID["FR_0"]])
        # print("IMU state: ", msg.imu_state)
        # print("Battery state: voltage: ", msg.power_v, "current: ", msg.power_a)

    def euler_from_quaternion(self, x, y, z, w):
        """
        https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def PoseMessageHandler(self, msg: PoseStamped_):
        self.pose = msg

        logging.info(f"{msg}")

        self.body_height_cm = round(self.pose.pose.position.z * 100.0)

        if self.body_attitude_previous is None:
            if self.body_height_cm < 24:
                self.body_attitude_previous = "sitting"
            else:
                self.body_attitude_previous = "standing"

        x = self.pose.pose.orientation.x
        y = self.pose.pose.orientation.y
        z = self.pose.pose.orientation.z
        w = self.pose.pose.orientation.w

        angles = self.euler_from_quaternion(x, y, z, w)

        self.yaw_now = angles[2] * rad_to_deg * -1.0

        # we set CW yaw = positive
        # this runs from -180 to +180

        # current position in world frame
        self.x = self.pose.pose.position.x
        self.y = self.pose.pose.position.y

        logging.info(
            f"NAV x,y,yaw: {round(self.x,2)},{round(self.y,2)},{round(self.yaw_now,2)}"
        )


    def start(self):
        """
        Starts the Navigation Provider and processing thread
        if not already running.
        """
        if self._thread and self._thread.is_alive():
            return

        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # def _preprocess_zenoh(self, scan):

    #     if scan is None:
    #         logging.info("waiting for Zenoh Laserscan data...")
    #         self._raw_scan = []
    #         self._lidar_string = "You might be surrounded by objects and cannot safely move in any direction. DO NOT MOVE."
    #         self._valid_paths = []
    #     else:
    #         logging.debug(f"_preprocess_zenoh: {scan}")
    #         # angle_min=-3.1241390705108643, angle_max=3.1415927410125732
    #         angles = list(
    #             map(
    #                 lambda x: 360.0 * (x + math.pi) / (2 * math.pi),
    #                 np.arange(scan.angle_min, scan.angle_max, scan.angle_increment),
    #             )
    #         )

    #         angles_final = np.flip(angles)
    #         # angles now run from 360.0 to 0 degress
    #         data = list(zip(angles_final, scan.ranges))
    #         array_ready = np.array(data)
    #         # print(f"Array {array_ready}")
    #         self._process(array_ready)

    # def _preprocess_serial(self, scan):
    #     logging.debug(f"_preprocess_serial: {scan}")
    #     array = np.array(scan)

    #     # the driver sends angles in degrees between from 0 to 360
    #     # warning - the driver may send two or more readings per angle,
    #     # this can be confusing for the code
    #     angles = array[:, 1]

    #     # distances are in millimeters
    #     distances_mm = array[:, 2]
    #     distances_m = [i / 1000 for i in distances_mm]

    #     data = list(zip(angles, distances_m))
    #     array_ready = np.array(data)
    #     # print(f"Array {array_ready}")
    #     self._process(array_ready)

    # def _process(self, data):

    #     complexes = []

    #     for angle, distance in data:

    #         d_m = distance

    #         # don't worry about distant objects
    #         if d_m > 5.0:
    #             continue

    #         # first, correctly orient the sensor zero to the robot zero
    #         angle = angle + self.sensor_mounting_angle
    #         if angle >= 360.0:
    #             angle = angle - 360.0
    #         elif angle < 0.0:
    #             angle = 360.0 + angle

    #         # then, convert to radians
    #         a_rad = angle * math.pi / 180.0

    #         v1 = d_m * math.cos(a_rad)
    #         v2 = d_m * math.sin(a_rad)

    #         # convert to x and y
    #         # x runs backwards to forwards, y runs left to right
    #         x = -1 * v2
    #         y = -1 * v1

    #         # convert the angle to -180 to + 180 range
    #         angle = angle - 180.0

    #         keep = True
    #         for b in self.angles_blanked:
    #             if angle >= b[0] and angle <= b[1]:
    #                 # this is a permanent reflection based on the robot
    #                 # disregard
    #                 keep = False
    #                 break

    #         # the final data ready to use for path planning
    #         if keep:
    #             complexes.append([x, y, angle, d_m])

    #     array = np.array(complexes)

    #     # sort data into strictly increasing angles to deal with sensor issues
    #     # the sensor sometimes reports part of the previous scan and part of the next scan
    #     # so you end up with multiple slightly different values for some angles at the
    #     # junction
    #     sorted_indices = array[:, 2].argsort()
    #     array = array[sorted_indices]

    #     X = array[:, 0]
    #     Y = array[:, 1]
    #     # A = array[:, 2]
    #     D = array[:, 3]

    #     """
    #     Determine set of possible paths
    #     """
    #     possible_paths = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

    #     # all the possible conflicting points
    #     for x, y, d in list(zip(X, Y, D)):
    #         if d > self.max_relevant_distance:  # too far away - we do not care
    #             continue
    #         for apath in possible_paths:
    #             for point in self.pp[apath]:
    #                 p1 = x - point[0]
    #                 p2 = y - point[1]
    #                 dist = math.sqrt(p1 * p1 + p2 * p2)
    #                 if dist < self.half_width_robot:
    #                     # too close - this path will not work
    #                     path_to_remove = np.array([apath])
    #                     possible_paths = np.setdiff1d(possible_paths, path_to_remove)
    #                     break  # no need to keep checking this path - we know this path is bad

    #     turn_left = []
    #     advance = []
    #     turn_right = []
    #     retreat = []

    #     for p in possible_paths:
    #         if p < 4:
    #             turn_left.append(p)
    #         elif p == 4:
    #             advance.append(p)
    #         elif p < 9:
    #             turn_right.append(p)
    #         elif p == 9:
    #             retreat.append(p)

    #     return_string = "You are surrounded by objects and cannot safely move in any direction. DO NOT MOVE."

    #     if len(possible_paths) > 0:
    #         return_string = "The safe movement choices are: "
    #         if self.use_zenoh:  # i.e. you are controlling a TurtleBot4
    #             return_string += "You can turn left. You can turn right. "
    #             if len(advance) > 0:
    #                 return_string += "You can move forwards. "
    #         else:
    #             if len(turn_left) > 0:
    #                 return_string += "You can turn left. "
    #             if len(advance) > 0:
    #                 return_string += "You can move forwards. "
    #             if len(turn_right) > 0:
    #                 return_string += "You can turn right. "
    #             if len(retreat) > 0:
    #                 return_string += "You can move back. "

    #     self._raw_scan = array
    #     self._lidar_string = return_string
    #     self._valid_paths = possible_paths.tolist()

    #     logging.debug(
    #         f"RPLidar Provider string: {self._lidar_string}\nValid paths: {self._valid_paths}"
    #     )

    def _run(self):
        """
        Main loop for the Navigation provider.

        # Continuously processes RPLidar data and send them
        # to the inputs and actions, as needed.
        """
        while self.running:

            if self.use_zenoh:
                # global gNav
                logging.debug(f"Zenoh")
                # self._preprocess_zenoh(gNav)
            else:
                logging.info(
                    f"Cyclone Nav Provider string: {self.body_height_cm}"
                )
                # # we are using CycloneDDS
                # try:
                #     # for i, scan in enumerate(
                #     #     self.lidar.iter_scans(
                #     #         scan_type="express", max_buf_meas=3000, min_len=5
                #     #     )
                #     # ):
                #     #     self._preprocess_serial(scan)
                #     #     # not sure about the level of this?
                #     #     #time.sleep(0.1)
                # except Exception as e:
                #     logging.error(f"Error in Cyclone DDS provider: {e}")
            time.sleep(0.1)

    def stop(self):
        """
        Stop the Navigation provider.
        """
        self.running = False
        if self._thread:
            logging.info("Stopping Navigation provider")
            if not self.use_zenoh:
                # self.lidar.stop()
                # self.lidar.disconnect()
                time.sleep(0.5)
            self._thread.join(timeout=5)

    @property
    def position(self) -> Optional[list]:
        # """
        # Get the currently valid paths.

        # Returns
        # -------
        # Optional[list]
        #     The currently valid paths as a list, or None if not
        #     available. The list contains 0 to 10 entries,
        #     corresponding to possible paths - for example: [0,3,4,5]
        # """
        return self._position

