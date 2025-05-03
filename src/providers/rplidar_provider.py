import logging
import math
import threading
import time
from typing import Optional

import bezier
import numpy as np
from rpdriver_driver import RPDriver

from .singleton import singleton


@singleton
class RPLidarProvider:
    """
    RPLidar Provider.

    This class implements a singleton pattern to manage RPLidar data streaming.

    Parameters
    ----------
    serial_port : str
        The name of the serial port in use by the RPLidar sensor.
    """

    def __init__(self, serial_port: str = "/dev/cu.usbserial-0001"):
        """
        Robot and sensor configuration
        ToDo: move to config.json5 file
        """
        self.half_width_robot = 0.20  # the width of the robot is 40 cm
        self.max_relevant_distance = 1.10  # meters
        self.sensor_mounting_angle = 180.0  # corrects for how sensor is mounted

        self.running: bool = False
        self.lidar = None

        """
        precompute Bezier trajectories 
        """
        self.curves = [
            bezier.Curve(
                np.asfortranarray([[0.0, -0.3, -0.75], [0.0, 0.5, 0.40]]), degree=2
            ),
            bezier.Curve(
                np.asfortranarray([[0.0, -0.3, -0.70], [0.0, 0.6, 0.70]]), degree=2
            ),
            bezier.Curve(
                np.asfortranarray([[0.0, -0.2, -0.60], [0.0, 0.7, 0.90]]), degree=2
            ),
            bezier.Curve(
                np.asfortranarray([[0.0, -0.1, -0.35], [0.0, 0.7, 1.03]]), degree=2
            ),
            bezier.Curve(
                np.asfortranarray([[0.0, 0.0, 0.00], [0.0, 0.5, 1.05]]), degree=2
            ),
            bezier.Curve(
                np.asfortranarray([[0.0, +0.1, +0.35], [0.0, 0.7, 1.03]]), degree=2
            ),
            bezier.Curve(
                np.asfortranarray([[0.0, +0.2, +0.60], [0.0, 0.7, 0.90]]), degree=2
            ),
            bezier.Curve(
                np.asfortranarray([[0.0, +0.3, +0.70], [0.0, 0.6, 0.70]]), degree=2
            ),
            bezier.Curve(
                np.asfortranarray([[0.0, +0.3, +0.75], [0.0, 0.5, 0.40]]), degree=2
            ),
            bezier.Curve(
                np.asfortranarray([[0.0, 0.0, 0.00], [0.0, -0.5, -1.05]]), degree=2
            ),
        ]

        self.paths = []
        self.pp = []
        self.s_vals = np.linspace(0.0, 1.0, 10)

        for curve in self.curves:
            cp = curve.evaluate_multi(self.s_vals)
            self.paths.append(cp)
            pairs = list(zip(cp[0], cp[1]))
            self.pp.append(pairs)

        logging.info(self.paths)
        logging.info(self.pp)

        try:
            self.lidar = RPDriver(serial_port)

            info = self.lidar.get_info()
            ret = f"Info: {info}"
            print(ret)
            logging.info(ret)

            health = self.lidar.get_health()
            ret = f"Health: {health}"
            print(ret)
            logging.info(ret)

            # reset to clear buffers
            self.lidar.reset()
        except Exception as e:
            logging.error(f"Error in RPLidar provider: {e}")

        self._thread: Optional[threading.Thread] = None

    # def register_message_callback(self, message_callback: Optional[Callable]):
    #     """
    #     Register a callback for processing RPLidar results.

    #     Parameters
    #     ----------
    #     callback : callable
    #         The callback function to process VLM results.
    #     """
    #     self.ws_client.register_message_callback(message_callback)

    def start(self):
        """
        Start the RPLidar provider.

        Starts the RPLidar and processing thread
        if not already running.
        """
        if self._thread and self._thread.is_alive():
            return

        self.running = True
        self._thread = threading.Thread(
            target=self._run, daemon=True, args=(self.lidar,)
        )
        self._thread.start()

    def _run(self):
        """
        Main loop for the RPLidar provider.

        Continuously processes RPLidar data and sends them
        to the inputs and actions, as needed.
        """
        while self.running:
            try:
                for i, scan in enumerate(
                    self.lidar.iter_scans(
                        scan_type="express", max_buf_meas=3000, min_len=5
                    )
                ):

                    array = np.array(scan)

                    # the driver sends angles in degrees between from 0 to 360
                    angles = array[:, 1]

                    # distances are in millimeters
                    distances = array[:, 2]

                    complexes = []

                    for angle, distance in list(zip(angles, distances)):

                        # convert distance to meters
                        d_m = distance / 1000.0

                        # don't worry about distant objects
                        if d_m > 5.0:
                            continue

                        # first, correctly orient the sensor zero to the robot zero
                        # sensor_mounting_angle = 180.0
                        angle = angle + self.sensor_mounting_angle
                        if angle >= 360.0:
                            angle = angle - 360.0

                        # then, convert to radians
                        a_rad = angle * math.pi / 180.0

                        v1 = d_m * math.cos(a_rad)
                        v2 = d_m * math.sin(a_rad)

                        # convert to x and y
                        # x runs backwards to forwards, y runs left to right
                        x = -1 * v2
                        y = -1 * v1

                        # also, convert the angle to -180 to + 180 range
                        complexes.append([x, y, -180 + angle, d_m])

                    array = np.array(complexes)
                    X = array[:, 0]
                    Y = array[:, 1]
                    # A = array[:, 2]
                    D = array[:, 3]
                    # print(complexes)

                    """
                        Determine set of possible paths
                        """
                    possible_paths = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
                    bad_paths = []

                    # all the possible conflicting points
                    for x, y, d in list(zip(X, Y, D)):
                        if (
                            d > self.max_relevant_distance
                        ):  # too far away - we do not care
                            continue
                        for apath in possible_paths:
                            for point in self.pp[apath]:
                                p1 = x - point[0]
                                p2 = y - point[1]
                                dist = math.sqrt(p1 * p1 + p2 * p2)
                                if dist < self.half_width_robot:
                                    # too close - this path will not work
                                    path_to_remove = np.array([apath])
                                    bad_paths.append(apath)
                                    possible_paths = np.setdiff1d(
                                        possible_paths, path_to_remove
                                    )
                                    break  # no need to keep checking this path - we know this path is bad

                    # print(f"possible: {possible_paths}")

                    turn_left = []
                    turn_right = []
                    advance = []
                    retreat = []

                    for p in possible_paths:
                        # all the possible paths
                        if p < 4:
                            turn_left.append(p)
                        elif p == 4:
                            advance.append(p)
                        elif p < 9:
                            turn_right.append(p)
                        elif p == 9:
                            retreat.append(p)

                    return_string = ""

                    if len(possible_paths) > 0:
                        return_string += (
                            f"There are {len(possible_paths)} possible paths.\n"
                        )
                        if len(turn_left) > 0:
                            return_string += (
                                f"You can turn left using paths: {turn_left}.\n"
                            )
                        if len(advance) > 0:
                            return_string += "You can advance.\n"
                        if len(turn_right) > 0:
                            return_string += (
                                f"You can turn right using paths: {turn_right}.\n"
                            )
                        if len(retreat) > 0:
                            return_string += "You can retreat.\n"
                    else:
                        return_string = "You are surrounded by objects and cannot safely move in any direction. DO NOT MOVE."

                    logging.info(f"RPLidar result: {return_string}")

                    #####################################
                    # string to return: return_string
                    # array to return: array

                # time.sleep(0.1)
            except Exception as e:
                logging.error(f"Error in RPLidar provider: {e}")

    def stop(self):
        """
        Stop the RPLidar provider.

        Stops the websocket client, video stream, and processing thread.
        """
        self.running = False
        if self._thread:
            logging.info("Stopping RPLidar provider")
            self.lidar.stop()
            self.lidar.disconnect()
            time.sleep(0.5)
            self._thread.join(timeout=5)
