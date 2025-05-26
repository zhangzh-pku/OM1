import logging
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass

import requests

from .singleton import singleton


@dataclass
class FabricData:
    """
    Data class to represent a data submission from a machine to FABRIC.
    """

    machine_id: str
    gps_time: str
    lat: float
    lon: float
    alt: float
    odom_x: float
    odom_y: float
    odom_yaw: float

    def to_dict(self) -> dict:
        """
        Convert the FabricData object to a dictionary.

        Returns
        -------
        dict
            Dictionary representation of the FabricData object.
        """
        return {
            "machine_id": self.machine_id,
            "gps_time": self.gps_time,
            "vyaw": self.vyaw,
            "timestamp": self.timestamp,
        }


@singleton
class FabricDataSubmitter:
    """
    Allows a machine to submit data to FABRIC.
    """

    def __init__(
        self,
        api_key: str = None,
        base_url: str = "https://api.openmind.org/api/core/fabric/submit",
    ):
        """
        Initialize the MapDataProvider.

        Parameters
        ----------
        api_key : str
            API key for authentication. Default is None.
        base_url : str
            Base URL for the teleops status API. Default is
            "https://api.openmind.org/api/core/fabric/submit".
        """
        self.api_key = api_key
        self.base_url = base_url
        self.executor = ThreadPoolExecutor(max_workers=1)

    def _share_data_worker(self, data: FabricData):
        """
        Worker function to share data from the machine.
        This function runs in a separate thread to avoid blocking the main thread.

        Parameters
        ----------
        data : FabricData
            The status of the machine to be shared.
        """
        if self.api_key is None or self.api_key == "":
            logging.error("API key is missing. Cannot share data to FABRIC.")
            return

        try:
            request = requests.post(
                self.base_url,
                headers={"Authorization": f"Bearer {self.api_key}"},
                json=data.to_dict(),
            )

            if request.status_code == 200:
                logging.debug(f"Data shared successfully: {request.json()}")
            else:
                logging.error(
                    f"Failed to share data: {request.status_code} - {request.text}"
                )
        except Exception as e:
            logging.error(f"Error sharing data: {str(e)}")

    def share_data(self, data: FabricData):
        """
        Share mapping data.
        This function submits mapping data collected by a machine to a thread pool executor
        to run in a separate thread.

        Parameters
        ----------
        data : FabricData
            A mapping data payload to submit.
        """
        self.executor.submit(self._share_data_worker, data)
