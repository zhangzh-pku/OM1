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

# Cyclone DDS channel imports -------------------------------------------------
from unitree.unitree_sdk2py.core.channel import (
    ChannelSubscriber,
    ChannelFactoryInitialize,  # noqa: F401  (ensure DDS initialised elsewhere)
)
from unitree.unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

# ─── constants ───────────────────────────────────────────────────────────────
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

# ─── 1‑D Yaw Kalman filter ───────────────────────────────────────────────────
class YawKalman:
    """2‑state (yaw, yaw_rate) Kalman filter.

    Keeps yaw wrapped to (−π, π].  All angles in **radians** internally.
    """

    def __init__(
        self,
        var_yaw_init: float = math.radians(10) ** 2,
        var_rate_init: float = math.radians(2) ** 2,
        accel_var: float = math.radians(5) ** 2,   # angular acceleration²
        ypr_var: float = math.radians(8) ** 2,
        hdg_var: float = math.radians(3) ** 2,
        gyro_var: float = math.radians(1) ** 2,
    ):
        self.x = np.zeros((2, 1))                  # [θ, ω]
        self.P = np.diag([var_yaw_init, var_rate_init])
        self.q = accel_var
        self.Ry = np.array([[ypr_var]])            # IMU yaw obs noise
        self.Rh = np.array([[hdg_var]])            # compass noise
        self.Rg = np.array([[gyro_var]])           # yaw‑rate noise
        self.t0 = time.time()
        self.initialised = False

    def _set_initial(self, yaw_deg: float):
        """Force-initialise state with first reading."""
        self.x[0, 0] = math.radians(yaw_deg)
        self.x[1, 0] = 0.0                # assume 0 °/s at start
        self.initialised = True

    # –– helpers ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    @staticmethod
    def _wrap(a: float) -> float:
        """Wrap angle to (−π, π]."""
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _innovation(self, z: np.ndarray, h: np.ndarray) -> np.ndarray:
        e = z - h
        e[0, 0] = self._wrap(e[0, 0])
        return e

    # –– predict –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    def predict(self):
        t = time.time()
        dt = t - self.t0
        if dt <= 0:
            return
        self.t0 = t

        F = np.array([[1, dt], [0, 1]])
        G = np.array([[0.5 * dt ** 2], [dt]])
        Q = G @ np.array([[self.q]]) @ G.T

        self.x = F @ self.x
        self.x[0, 0] = self._wrap(self.x[0, 0])
        self.P = F @ self.P @ F.T + Q

    # –– updates –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    def _update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray):
        y = self._innovation(z, H @ self.x)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.x[0, 0] = self._wrap(self.x[0, 0])
        self.P = (np.eye(2) - K @ H) @ self.P

    def update_ypr(self, yaw_deg: float):
        if not self.initialised:
            self._set_initial(yaw_deg)
            return
        z = np.array([[math.radians(yaw_deg)]])
        self._update(z, np.array([[1, 0]]), self.Ry)

    def update_hdg(self, hdg_deg: float):
        if not self.initialised:
            self._set_initial(hdg_deg)
            return
        z = np.array([[math.radians(hdg_deg)]])
        self._update(z, np.array([[1, 0]]), self.Rh)

    def update_gyro(self, rate_rad_s: float):
        z = np.array([[rate_rad_s]])
        self._update(z, np.array([[0, 1]]), self.Rg)

    # –– outputs –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    @property
    def yaw_deg(self) -> float:
        return math.degrees(self.x[0, 0])

    @property
    def yaw_rate_deg_s(self) -> float:
        return math.degrees(self.x[1, 0])


# ─── 4‑state EKF for (x, y, vx, vy) ─────────────────────────────────────────
class GPSOdomEKF:
    """Odometry‑corrected GPS position filter (unchanged)."""

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
        self.x = np.zeros((4, 1))
        self.P = np.diag([pos_var_init, pos_var_init, vel_var_init, vel_var_init])

        self.q_acc = acc_var
        self.R_gps = np.diag([gps_pos_var, gps_pos_var])
        self.R_odo = np.diag([odo_vel_var, odo_vel_var])

        self.lat0 = math.radians(origin_lat)
        self.lon0 = math.radians(origin_lon)
        self.R_e = 6_371_000.0  # mean Earth radius (m)

    # –– helpers –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    def _latlon_to_xy(self, lat: float, lon: float):
        φ, λ = map(math.radians, (lat, lon))
        x = self.R_e * (λ - self.lon0) * math.cos(self.lat0)
        y = self.R_e * (φ - self.lat0)
        return np.array([[x], [y]])

    def _xy_to_latlon(self, x: float, y: float):
        φ = self.lat0 + y / self.R_e
        λ = self.lon0 + x / (self.R_e * math.cos(self.lat0))
        return map(math.degrees, (φ, λ))

    # –– predict / updates ––––––––––––––––––––––––––––––––––––––––––––––––
    def predict(self, dt: float):
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        G = np.array([[0.5 * dt ** 2, 0], [0, 0.5 * dt ** 2], [dt, 0], [0, dt]])
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

    # –– accessors ––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    @property
    def xy(self):
        return self.x[0, 0], self.x[1, 0]

    @property
    def latlon(self):
        return self._xy_to_latlon(*self.xy)


# ─── helper dataclass for buffering to LLM ─────────────────────────────────--
@dataclass
class Message:
    """
    Container for timestamped messages.

    Parameters
    ----------
    timestamp : float
        Unix timestamp of the message
    text : str
        Content of the message
    """

    timestamp: float
    text: str


# ─── Serial + DDS reader/fuser ───────────────────────────────────────────────
class GPSMagSerialReader(FuserInput[str]):
    """Polls serial GPS NMEA + Unitree odometry and fuses with EKF if available.

    Origin latitude and longitude can now be supplied via `SensorConfig` as
    `origin_lat` and `origin_lon` to avoid using the first GPS fix as origin.
    """

    def __init__(self, config: SensorConfig = SensorConfig()):
        super().__init__(config)

        self._return_state = 0            # rotate YPR → HDG → GPS
        self.descriptor_for_LLM = "Location and Velocity"

        # — origin selection ------------------------------------------------
        self.origin_lat = getattr(config, "origin_lat", None)
        self.origin_lon = getattr(config, "origin_lon", None)
        self.pending_origin: Optional[tuple[float, float]] = None
        if self.origin_lat is not None and self.origin_lon is not None:
            self.pending_origin = (self.origin_lat, self.origin_lon)
            logging.info(f"Using origin from config: {self.pending_origin}")

        # — filters ---------------------------------------------------------
        self.ekf: Optional[GPSOdomEKF] = None
        self.yaw_kf = YawKalman()
        self.filter_enabled = False
        self.last_odom: Optional[SportModeState_] = None

        # — DDS subscription to sportmode ----------------------------------
        self.odom_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.odom_sub.Init(self._odom_cb, 10)

        # — serial port -----------------------------------------------------
        try:
            self.ser = serial.Serial(getattr(config, "port", None), 115200, timeout=1)
            logging.info("Opened GPS serial")
        except Exception as e:
            logging.error(f"GPS serial open error: {e}")
            self.ser = None

        # — misc ------------------------------------------------------------
        self.t_last_predict = time.time()
        self.io_provider = IOProvider()
        self.buf: list[Message] = []

    # –– DDS callback –––––––––––––––––––––––––––––––––––––––––––––––––––––––
    def _odom_cb(self, msg: SportModeState_):
        self.last_odom = msg
        if not self.filter_enabled:
            self.filter_enabled = True
        if self.filter_enabled and self.ekf is None and self.pending_origin:
            lat0, lon0 = self.pending_origin
            self.ekf = GPSOdomEKF(lat0, lon0)
            logging.info("EKF instantiated after odometry became available")

    # –– prediction step ––––––––––––––––––––––––––––––––––––––––––––––––––––
    async def _predict_step(self):
        # Heading filter (always running)
        self.yaw_kf.predict()

        # Position EKF only when enabled -----------------------------------
        if not (self.filter_enabled and self.ekf):
            return
        now = time.time()
        dt = now - self.t_last_predict
        self.t_last_predict = now
        self.ekf.predict(dt)

        if self.last_odom is not None:
            try:
                vx, vy, _ = self.last_odom.velocity
                self.ekf.update_odom(vx, vy)

                # Feed gyro yaw‑rate to KF (already in rad/s)
                self.yaw_kf.update_gyro(self.last_odom.yaw_speed)
            except Exception as e:
                logging.warning(f"Odom update failed: {e}")

    # –– polling loop –––––––––––––––––––––––––––––––––––––––––––––––––––––––
    async def _poll(self) -> Optional[str]:
        await asyncio.sleep(0.5)           # keep cadence
        await self._predict_step()

        if not self.ser:
            return None

        newest_any = newest_ypr = newest_hdg = newest_gps = None

        while self.ser.in_waiting:
            raw = self.ser.readline().decode("utf-8", "ignore").strip()
            newest_any = raw
            if raw.startswith("YPR"):
                newest_ypr = raw
            elif raw.startswith("HDG"):
                newest_hdg = raw
            elif raw.startswith("GPS:"):
                newest_gps = raw

        choice_map = {0: newest_ypr, 1: newest_hdg, 2: newest_gps}
        chosen = choice_map[self._return_state] or newest_any
        self._return_state = (self._return_state + 1) % 3

        if chosen:
            logging.info(f"Serial: {chosen}")
        return chosen

    # –– parsing ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    async def _raw_to_text(self, raw: str) -> Message:
        now = time.time()
        txt = "Unrecognised data"

        try:
            # -------------------- GPS -------------------------------------
            if raw.startswith("GPS:"):
                parts = raw[4:].split(",")
                lat = float(parts[0][:-1]) * (1 if parts[0].endswith("N") else -1)
                lon = float(parts[1][:-1]) * (1 if parts[1].endswith("E") else -1)

                if self.pending_origin is None and not (self.origin_lat and self.origin_lon):
                    self.pending_origin = (lat, lon)

                if self.filter_enabled and self.ekf:
                    self.ekf.update_gps(lat, lon)
                    x, y = self.ekf.xy
                    vx, vy = self.ekf.x[2, 0], self.ekf.x[3, 0]
                    txt = f"GPS→({x:.2f} m, {y:.2f} m)  vel=({vx:.2f},{vy:.2f}) m/s"
                    lat_f, lon_f = self.ekf.latlon
                    self.io_provider.add_dynamic_variable("latitude", lat_f)
                    self.io_provider.add_dynamic_variable("longitude", lon_f)
                else:
                    txt = f"GPS raw → {lat:.5f}°, {lon:.5f}°"
                    self.io_provider.add_dynamic_variable("latitude", lat)
                    self.io_provider.add_dynamic_variable("longitude", lon)

            # -------------------- HDG -------------------------------------
            elif raw.startswith("HDG (DEG):"):
                parts = raw.split()
                if len(parts) >= 4:
                    heading_deg = float(parts[2])
                    cardinal = CARDINAL_MAP.get(parts[3], parts[3])
                    self.yaw_kf.update_hdg(heading_deg)
                    fused = self.yaw_kf.yaw_deg
                    txt = f"You are facing {cardinal} ({fused:.2f}°)."
                    self.io_provider.add_dynamic_variable("direction", cardinal)
                    self.io_provider.add_dynamic_variable("heading_deg", heading_deg)
                else:
                    txt = f"Unable to parse heading: {raw}"

            # -------------------- YPR -------------------------------------
            elif raw.startswith("YPR:"):
                try:
                    yaw_s, pitch_s, roll_s = map(str.strip, raw[4:].split(","))
                    yaw, pitch, roll = map(float, (yaw_s, pitch_s, roll_s))
                    self.yaw_kf.update_ypr(yaw)
                    fused = self.yaw_kf.yaw_deg
                    txt = (
                        f"Orientation YPR=({yaw:.1f}°, {pitch:.1f}°, {roll:.1f}°)  | "
                    )
                    for k, v in zip(("yaw", "pitch", "roll"), (yaw, pitch, roll)):
                        self.io_provider.add_dynamic_variable(k, v)
                except Exception as e:
                    txt = f"Failed to parse YPR: {e}"

            # -------------------- default ---------------------------------
            else:
                txt = raw

            # expose fused yaw each cycle (even if this line was GPS only)
            logging.info(f"Fused yaw: {self.yaw_kf.yaw_deg:.2f}°")
            self.io_provider.add_dynamic_variable("fused_yaw_deg", self.yaw_kf.yaw_deg)
            self.io_provider.add_dynamic_variable("yaw_rate_deg_s", self.yaw_kf.yaw_rate_deg_s)

        except Exception as e:
            txt = f"Parse error: {e}"
            logging.error(txt)

        self.io_provider.add_input(self.__class__.__name__, txt, now)
        return Message(timestamp=now, text=txt)

    # public helper -----------------------------------------------------------
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
        self.io_provider.add_input(self.__class__.__name__, m.text, m.timestamp)
        return f"""
{self.descriptor_for_LLM} INPUT
// START
{m.text}
// END
"""
