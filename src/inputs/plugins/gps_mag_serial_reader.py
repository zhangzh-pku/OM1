#!/usr/bin/env python3
import asyncio
import logging
import time
import math
from typing import Optional
import serial
import numpy as np

from inputs.base import SensorConfig
from inputs.base.loop import FuserInput
from providers.io_provider import IOProvider

# SportClient from unitree package
from unitree.unitree_sdk2py.go2.sport.sport_client import SportClient


# ——— 3‑state EKF for [x, y, θ] ——————————————————————————————————————————————————
class GPSOdomEKF:
    def __init__(
        self,
        origin_lat: float,
        origin_lon: float,
        init_var: float = 1.0,
        var_v: float = 0.1,
        var_omega: float = 0.1,
        gps_var: float = 5.0,
    ):
        self.x = np.zeros((3, 1))  # [x, y, θ]
        self.P = np.eye(3) * init_var

        self.var_v = var_v
        self.var_omega = var_omega
        self.R_gps = np.eye(2) * gps_var

        # ENU origin
        self.lat0 = math.radians(origin_lat)
        self.lon0 = math.radians(origin_lon)
        self.R_e = 6371000.0

    def latlon_to_xy(self, lat: float, lon: float) -> np.ndarray:
        φ = math.radians(lat)
        λ = math.radians(lon)
        x = self.R_e * (λ - self.lon0) * math.cos(self.lat0)
        y = self.R_e * (φ - self.lat0)
        return np.array([[x], [y]])

    def predict(self, dt: float, v: float, omega: float) -> None:
        θ = self.x[2, 0]
        F = np.array(
            [[1, 0, -v * math.sin(θ) * dt], [0, 1, v * math.cos(θ) * dt], [0, 0, 1]]
        )
        V = np.array([[math.cos(θ) * dt, 0], [math.sin(θ) * dt, 0], [0, dt]])
        self.x[0, 0] += v * math.cos(θ) * dt
        self.x[1, 0] += v * math.sin(θ) * dt
        # θ unchanged here

        M = np.diag([self.var_v, self.var_omega])
        Q = V @ M @ V.T
        self.P = F @ self.P @ F.T + Q

    def update_gps(self, lat: float, lon: float) -> None:
        z = self.latlon_to_xy(lat, lon)
        H = np.array([[1, 0, 0], [0, 1, 0]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

    @property
    def state(self):
        return self.x.flatten().tolist()


# ——— GPS+Odometry Reader with EKF —————————————————————————————————
from dataclasses import dataclass


@dataclass
class Message:
    """
    Container for timestamped messages.

    Parameters
    ----------
    timestamp : float
        Unix timestamp of the message
    message : str
        Content of the message
    """

    timestamp: float
    message: str


class GPSMagSerialReader(FuserInput[str]):
    """
    Reads GPS serial lines, polls Unitree SportClient for odom,
    fuses in EKF, and exposes filtered (x,y) via IOProvider.
    """

    def __init__(self, config: SensorConfig = SensorConfig()):
        super().__init__(config)

        port = getattr(config, "port", None)
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            logging.info(f"Opened GPS serial on {port}")
        except Exception as e:
            logging.error(f"GPS serial open error: {e}")
            self.ser = None

        self.sport = SportClient()
        self.sport.Init()

        self.ekf = None
        self.origin_set = False
        self.last_time = time.time()

        self.io_provider = IOProvider()
        self.messages = []
        self.descriptor_for_LLM = "Location and Orientation"

    async def _poll(self) -> Optional[str]:
        await asyncio.sleep(0.5)
        if not self.ser:
            return None
        line = self.ser.readline().decode(errors="ignore").strip()
        return line or None

    async def _raw_to_text(self, raw: str) -> Message:
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # EKF predict with odometry
        if self.ekf:
            code, data = self.sport.GetState(["Position", "Velocity"])
            if code == 0:
                vx, vy, _ = data["Velocity"]
                v = math.hypot(vx, vy)
                self.ekf.predict(dt, v, 0.0)
                x_p, y_p, _ = self.ekf.state
                self.io_provider.add_dynamic_variable("pred_x", x_p)
                self.io_provider.add_dynamic_variable("pred_y", y_p)

        msg = "Unrecognized data"
        try:
            if raw.startswith("GPS:"):
                parts = raw[4:].split(",")
                lat = float(parts[0][:-1]) * (1 if parts[0].endswith("N") else -1)
                lon = float(parts[1][:-1]) * (1 if parts[1].endswith("E") else -1)
                sats = int(parts[5].split(":")[1])

                if not self.origin_set:
                    self.ekf = GPSOdomEKF(lat, lon)
                    self.origin_set = True

                self.ekf.update_gps(lat, lon)
                x_f, y_f, _ = self.ekf.state
                self.io_provider.add_dynamic_variable("filt_x", x_f)
                self.io_provider.add_dynamic_variable("filt_y", y_f)

                msg = f"GPS {lat:.6f},{lon:.6f} sats={sats} → filt=({x_f:.2f}m,{y_f:.2f}m)"
        except Exception as e:
            msg = f"Parse error [{raw}]: {e}"

        return Message(timestamp=now, message=msg)

    async def raw_to_text(self, raw: str):
        m = await self._raw_to_text(raw)
        if m:
            self.messages.append(m)

    def formatted_latest_buffer(self) -> Optional[str]:
        if not self.messages:
            return None
        m = self.messages[-1]
        out = f"""
{self.descriptor_for_LLM} INPUT
// START
{m.message}
// END
"""
        self.io_provider.add_input(self.__class__.__name__, m.message, m.timestamp)
        self.messages.clear()
        return out
