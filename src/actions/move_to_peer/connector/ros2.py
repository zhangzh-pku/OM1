import logging
import math

from actions.base import ActionConfig, ActionConnector
from providers.io_provider import IOProvider

from ..interface import MoveToPeerAction, MoveToPeerInput


class MoveToPeerRos2Connector(ActionConnector[MoveToPeerInput]):
    def __init__(self, config: ActionConfig):
        super().__init__(config)
        self.io = IOProvider()
        # init sport client as before…
        from unitree.unitree_sdk2py.go2.sport.sport_client import SportClient

        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

    async def connect(self, input_protocol: MoveToPeerInput) -> None:
        # check desired action
        if input_protocol.action == MoveToPeerAction.IDLE:
            logging.info("MoveToPeer: idle, no movement commanded.")
            return

        # else NAVIGATE
        lat0 = self.io.get_dynamic_variable("latitude")
        lon0 = self.io.get_dynamic_variable("longitude")
        lat1 = self.io.get_dynamic_variable("closest_peer_lat")
        lon1 = self.io.get_dynamic_variable("closest_peer_lon")

        # Ensure lat0 and lon0 are available
        if lat0 is None or lon0 is None:
            logging.info("MoveToPeer: own location not available, not moving.")
            return
        
        lat0 = float(lat0)
        lon0 = float(lon0)

        # Ensure closest_peer_lat and closest_peer_lon are available
        if lat1 is None or lon1 is None:
            logging.info("MoveToPeer: closest peer location not available, not moving.")
            return

        lat1 = float(lat1)
        lon1 = float(lon1)

        # compute approximate bearing vector in meters
        R = 6371000
        dlat = math.radians(lat1 - lat0)
        dlon = math.radians(lon1 - lon0)
        x = dlon * math.cos(math.radians((lat0 + lat1) / 2)) * R
        y = dlat * R
        distance = math.hypot(x, y)
        if distance < 4.0:
            logging.info(
                f"MoveToPeer: too close to peer, distance={distance:.1f}m, not moving."
            )
            return
        norm = distance or 1
        vx = (y / norm) * 0.5
        vy = (x / norm) * 0.5

        logging.info(
            f"MoveToPeer: moving → Δx={x:.1f}m Δy={y:.1f}m  vx={vx:.2f}, vy={vy:.2f}"
        )
        self.sport_client.Move(vx, vy, 0.0)

