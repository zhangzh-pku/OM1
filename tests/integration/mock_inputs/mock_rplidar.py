import asyncio
import logging
import time
from typing import Optional

from inputs.base import SensorConfig
from inputs.plugins.rplidar import RPLidar
from tests.integration.mock_inputs.data_providers.mock_lidar_provider import (
    get_lidar_provider,
    get_next_lidar_scan,
)


class MockRPLidar(RPLidar):
    """
    Mock implementation of RPLidar that uses mock lidar data.

    This class overrides only the lidar data input functionality to provide
    mock lidar data, while maintaining all the real processing logic.
    """

    def __init__(self, config: SensorConfig = SensorConfig()):
        """
        Initialize with the real RPLidar implementation but with mock data source.

        Parameters
        ----------
        config : SensorConfig, optional
            Configuration for the sensor
        """
        # Initialize using the real RPLidar implementation
        super().__init__(config)

        # Override the descriptor to indicate this is a mock
        self.descriptor_for_LLM = "MOCK RPLidar INPUT (Integration Test)"

        # Stop the original lidar provider
        if hasattr(self, "lidar") and self.lidar:
            # Note: Add proper cleanup for the original lidar if it has a stop method
            logging.info("MockRPLidar: Stopped original lidar provider")

        # Store the last processed time to rate-limit our mock data
        self.last_processed_time = 0

        # Track if we've processed all mock data
        self.mock_data_processed = False

        # Start the mock data processing
        self.running = True

        # Get reference to the lidar provider
        self.lidar_provider = get_lidar_provider()

        logging.info(f"MockRPLidar initialized with {self.lidar_provider.scan_count} scan data sets")

    async def _poll(self) -> Optional[str]:
        """
        Override the poll method to return mock data instead of real lidar data.

        Returns
        -------
        Optional[str]
            Mock lidar data string if available, None otherwise
        """
        await asyncio.sleep(0.2)  # Maintain the same polling rate

        # Rate limit to avoid overwhelming the system
        current_time = time.time()
        if current_time - self.last_processed_time < 0.5:  # Half second between data
            return None

        self.last_processed_time = current_time

        # Get next scan from the provider
        scan_array = get_next_lidar_scan()
        if scan_array is not None:
            # Process the scan data through the path processor
            if hasattr(self.lidar, '_path_processor'):
                self.lidar._path_processor(scan_array)
            
            logging.info(f"MockRPLidar: Processed mock scan ({self.lidar_provider.remaining_scans} remaining)")
            
            # Return the processed lidar string
            return self.lidar.lidar_string
        else:
            if not self.mock_data_processed:
                logging.info("MockRPLidar: No more mock scan data to process")
                self.mock_data_processed = True
            return None

    def cleanup(self):
        """
        Synchronous cleanup method for proper resource cleanup.
        """
        try:
            self.running = False

            # Clean up the original lidar provider if needed
            if hasattr(self, "lidar") and self.lidar:
                # Add proper cleanup for the lidar provider if it has cleanup methods
                pass

            logging.info("MockRPLidar: Cleanup completed")
        except Exception as e:
            logging.error(f"MockRPLidar: Error during cleanup: {e}")

    def __del__(self):
        """Clean up resources when the object is destroyed."""
        self.cleanup()

    def reset_mock_data(self):
        """
        Reset the mock data to start from the beginning.
        Useful for repeated testing.
        """
        self.lidar_provider.reset()
        self.mock_data_processed = False
        logging.info("MockRPLidar: Mock data reset")
