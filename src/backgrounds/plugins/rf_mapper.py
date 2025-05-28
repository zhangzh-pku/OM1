import asyncio
import logging
import threading
import time

from bleak import AdvertisementData, BleakScanner

from backgrounds.base import Background, BackgroundConfig
from providers.fabric_map_provider import FabricData, FabricDataSubmitter
from providers.gps_provider import GpsProvider


class RFmapper(Background):
    """
    Example background implementation that runs indefinitely.
    """

    def __init__(self, config: BackgroundConfig):
        """
        Initialize the RFmapper with configuration.

        Parameters
        ----------
        config : BackgroundConfig
            Configuration object for the background.
        """
        super().__init__(config)
        self.name = getattr(config, "name", "RFmapper")
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._scan_task)
        self.running = False
        self.json_payload = None
        self.scan_results = None
        self.gps_data = None

        self.gps = GpsProvider()
        self.gps_on = self.gps.running

        # self.silent = getattr(config, "silent", False)

        self.fds = FabricDataSubmitter(api_key="", write_to_local_file=True)

        self.start()

    async def scan_once(self):

        seen_devices = {}

        def detection_callback(device, advertisement_data: AdvertisementData):
            seen_devices[device.address] = {
                "timestamp": time.time(),
                "address": device.address,
                "name": device.name,
                "rssi": advertisement_data.rssi,
            }

        scanner = BleakScanner(detection_callback)
        await scanner.start()
        await asyncio.sleep(1.0)
        await scanner.stop()

        # Get the top 10 devices with the strongest RSSI
        sorted_devices = sorted(
            seen_devices.values(), key=lambda d: d["rssi"], reverse=True
        )[:10]
        return sorted_devices

    def _scan_task(self):
        asyncio.set_event_loop(self.loop)
        logging.info("Starting RF scan thread...")
        self.running = True
        while self.running:
            self.scan_results = self.loop.run_until_complete(self.scan_once())
            time.sleep(1)

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def run(self) -> None:
        """
        Run the background process.
        This method will run indefinitely, simulating a long-running task.
        """
        try:
            while True:
                # logging.info(f"RF mapper: {self.scan_results}")

                if hasattr(self.gps, "running"):
                    if self.scan_results and self.gps.data:
                        # self.scan_results.append(self.gps.data)
                        # self.json_payload = json.dumps(self.scan_results, indent=2)
                        # logging.info(f"Mapper data: {self.json_payload}")
                        g = self.gps.data
                        logging.debug(f"GPS data: {g}")
                        self.fds.share_data(
                            FabricData(
                                machine_id="Go2LA",
                                gps_time_utc=g["gps_time_utc"],
                                gps_lat=g["gps_lat"],
                                gps_lon=g["gps_lon"],
                                gps_alt=g["gps_alt"],
                                update_time_local=time.time(),
                                odom_x=0.0,  # TODO
                                odom_y=0.0,  # TODO
                                odom_yaw=0.0,  # TODO
                                rf_data=self.scan_results,
                            )
                        )
                        self.scan_results = None

                time.sleep(1)
        except KeyboardInterrupt:
            logging.info("Stopping RF scanner...")
            self.stop()
