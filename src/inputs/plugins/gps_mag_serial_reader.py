#!/usr/bin/env python3
import asyncio
import logging
import math
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import serial

from inputs.base import SensorConfig
from inputs.base.loop import FuserInput
from providers.io_provider import IOProvider

# Cyclone DDS channel imports
from unitree.unitree_sdk2py.core.channel import (
    ChannelSubscriber,
    ChannelFactoryInitialize,
)
from unitree.unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

CARDINAL_MAP = {
    "N": "North",
    "NNE": "North-Northeast",
    "NE": "Northeast",
    "ENE": "East-Northeast",
    "E": "East",
    "ESE": "East-Southeast",
    "SE": "Southeast",
    "SSE": "South-Southeast",
    "S": "South",
    "SSW": "South-Southwest",
    "SW": "Southwest",
    "WSW": "West-Southwest",
    "W": "West",
    "WNW": "West-Northwest",
    "NW": "Northwest",
    "NNW": "North-Northwest",
}


# ─── EKF implementation ───────────────────────────────────────────────────────────
class GPSOdomEKF:
    """4‑state EKF where odometry *corrects* GPS‑predicted motion."""

    def __init__(
        self,
        origin_lat: float,
        origin_lon: float,
        pos_var_init: float = 25.0,
        vel_var_init: float = 1.0,
        acc_var: float = 0.5,
        gps_pos_var: float = 25.0,
        odo_vel_var: float = 0.05,
    ):
        # state: [x, y, vx, vy]ᵀ (m, m, m/s, m/s)
        self.x = np.zeros((4, 1))
        self.P = np.diag([pos_var_init, pos_var_init, vel_var_init, vel_var_init])

        self.q_acc = acc_var  # process noise (m/s²)
        self.R_gps = np.diag([gps_pos_var, gps_pos_var])
        self.R_odo = np.diag([odo_vel_var, odo_vel_var])

        # origin for ENU conversion
        self.lat0 = math.radians(origin_lat)
        self.lon0 = math.radians(origin_lon)
        self.R_e = 6_371_000.0  # mean Earth radius

    # ── helpers ───────────────────────────────────────────────────────────────────
    def _latlon_to_xy(self, lat: float, lon: float):
        φ = math.radians(lat)
        λ = math.radians(lon)
        x = self.R_e * (λ - self.lon0) * math.cos(self.lat0)
        y = self.R_e * (φ - self.lat0)
        return np.array([[x], [y]])

    def _xy_to_latlon(self, x: float, y: float):
        φ = self.lat0 + y / self.R_e
        λ = self.lon0 + x / (self.R_e * math.cos(self.lat0))
        return math.degrees(φ), math.degrees(λ)

    # ── EKF core ───────────────────────────────────────────────────────────────────
    def predict(self, dt: float):
        # State‑transition & process noise
        F = np.array(
            [
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        G = np.array(
            [
                [0.5 * dt**2, 0],
                [0, 0.5 * dt**2],
                [dt, 0],
                [0, dt],
            ]
        )
        Q = G @ (self.q_acc * np.eye(2)) @ G.T

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update_gps(self, lat: float, lon: float):
        z = self._latlon_to_xy(lat, lon)
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

    def update_odom(self, vx: float, vy: float):
        z = np.array([[vx], [vy]])
        H = np.array([[0, 0, 1, 0], [0, 0, 0, 1]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_odo
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

    # ── convenient accessors ──────────────────────────────────────────────────────
    @property
    def xy(self):
        return self.x[0, 0], self.x[1, 0]

    @property
    def latlon(self):
        return self._xy_to_latlon(*self.xy)


# ─── Reader/Fuser ─────────────────────────────────────────────────────────────────
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
    text: str


class GPSMagSerialReader(FuserInput[str]):
    """Polls serial GPS NMEA + Unitree odometry and fuses with EKF."""

    def __init__(self, config: SensorConfig = SensorConfig()):
        super().__init__(config)

        # DDS – subscribe to odometry
        self.odom_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.odom_sub.Init(self._odom_cb, 10)
        self.last_odom: Optional[SportModeState_] = None

        # Serial – GPS
        port = getattr(config, "port", None)
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            logging.info(f"Opened GPS serial on {port}")
        except Exception as e:
            logging.error(f"GPS serial open error: {e}")
            self.ser = None

        # EKF
        self.ekf: Optional[GPSOdomEKF] = None
        self.origin_set = False

        # timing
        now = time.time()
        self.t_last_predict = now
        self.t_last_gps = now

        # misc
        self.io_provider = IOProvider()
        self.buf: list[Message] = []
        self.descriptor_for_LLM = "Location and Velocity"

    # ── DDS callback ─────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: SportModeState_):
        self.last_odom = msg
        # velocity update is done in *_predict_step below to keep timing consistent

    # ── internal helpers ─────────────────────────────────────────────────────────
    async def _predict_step(self):
        if not self.ekf:
            return
        now = time.time()
        dt = now - self.t_last_predict
        self.t_last_predict = now
        self.ekf.predict(dt)

        # correct velocity with most recent odometry
        if self.last_odom is not None:
            try:
                vx, vy, _ = self.last_odom.velocity
                self.ekf.update_odom(vx, vy)
            except Exception as e:
                logging.warning(f"Odom update failed: {e}")

    # ── serial polling ───────────────────────────────────────────────────────────
    async def _poll(self) -> Optional[str]:
        # predict every loop even if no new GPS line
        await self._predict_step()

        if not self.ser:
            await asyncio.sleep(0.1)
            return None
        line = self.ser.readline().decode(errors="ignore").strip()
        return line or None

    async def _raw_to_text(self, raw: str) -> Message:
        now = time.time()
        txt = "Unrecognised data"

        try:
            if raw.startswith("GPS:"):
                parts = raw[4:].split(",")
                lat = float(parts[0][:-1]) * (1 if parts[0].endswith("N") else -1)
                lon = float(parts[1][:-1]) * (1 if parts[1].endswith("E") else -1)

                if not self.origin_set:
                    self.ekf = GPSOdomEKF(lat, lon)
                    self.origin_set = True
                    logging.info("EKF origin established")

                self.ekf.update_gps(lat, lon)
                x, y = self.ekf.xy
                vx, vy = self.ekf.x[2, 0], self.ekf.x[3, 0]
                txt = f"GPS fix → ({x:.2f} m, {y:.2f} m)  vel=({vx:.2f},{vy:.2f}) m/s"

                lat_f, lon_f = self.ekf.latlon
                self.io_provider.add_dynamic_variable("latitude", lat_f)
                self.io_provider.add_dynamic_variable("longitude", lon_f)
                logging.info(
                    f"GPS: {lat_f:.5f} N, {lon_f:.5f} E  vel=({vx:.2f},{vy:.2f}) m/s"
                )

            elif raw.startswith("HDG (DEG):"):
                parts = raw.split()
                if len(parts) >= 4:
                    cardinal_abbr = parts[3]
                    direction = CARDINAL_MAP.get(cardinal_abbr, cardinal_abbr)
                    txt = f"You are facing {direction}."
                    self.io_provider.add_dynamic_variable("direction", direction)
                else:
                    txt = f"Unable to parse heading: {raw}"

            elif raw.startswith("YPR:"):
                try:
                    yaw, pitch, roll = map(str.strip, raw[4:].split(","))
                    txt = f"Orientation is Yaw: {yaw}°, Pitch: {pitch}°, Roll: {roll}°."
                    self.io_provider.add_dynamic_variable("yaw", yaw)
                    self.io_provider.add_dynamic_variable("pitch", pitch)
                    self.io_provider.add_dynamic_variable("roll", roll)
                except Exception as e:
                    txt = f"Failed to parse YPR: {e}"

            else:
                txt = raw  # pass through any other message

        except Exception as e:
            txt = f"Parse error: {e}"
            logging.error(txt)

        self.io_provider.add_input(self.__class__.__name__, txt, now)
        return Message(timestamp=now, text=txt)

    async def raw_to_text(self, raw: str):
        if not raw:
            return
        m = await self._raw_to_text(raw)
        self.buf.append(m)

    def formatted_latest_buffer(self) -> Optional[str]:
        if not self.buf:
            return None
        m = self.buf[-1]
        self.buf.clear()
        return f"""
{self.descriptor_for_LLM} INPUT
// START
{m.text}
// END
"""
