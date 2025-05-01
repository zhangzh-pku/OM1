import asyncio
import logging
from typing import Optional

import numpy as np

from inputs.base import SensorConfig
from inputs.plugins.vlm_coco_local import VLM_COCO_Local
from tests.integration.mock_inputs.mock_image_provider import get_next_opencv_image


class MockVLM_COCO(VLM_COCO_Local):
    """
    Mock implementation of VLM_COCO_Local that uses the central image provider.

    This class overrides only the camera input functionality to get images from
    the mock image provider, while maintaining all the real object detection logic.
    """

    def __init__(self, config: SensorConfig = SensorConfig()):
        """
        Initialize with the real VLM implementation but mock camera.

        Parameters
        ----------
        config : SensorConfig, optional
            Configuration for the sensor
        """
        # Initialize the parent class with the provided config
        super().__init__(config)

        # Override camera check
        self.have_cam = True  # Pretend we have a camera

        # Set up some dummy camera dimensions for the test
        self.width = 1280  # Standard webcam width
        self.height = 720  # Standard webcam height
        self.cam_third = int(self.width / 3)

        # Set a distinct descriptor to identify this as a mock
        self.descriptor_for_LLM = "MOCK Vision INPUT (COCO Test)"

        logging.info("MockVLM_COCO initialized - using mock image provider")

    async def _poll(self) -> Optional[np.ndarray]:
        """
        Override the camera polling to get images from the mock image provider.

        Returns
        -------
        Optional[np.ndarray]
            Next test image as a numpy array (OpenCV format), or None if no more images
        """
        await asyncio.sleep(0.01)  # Small delay to simulate real polling
        logging.info("MockVLM_COCO: Polling for image")

        # Get the next image from the central provider
        image = get_next_opencv_image()
        if image is not None:
            logging.info(f"MockVLM_COCO: Retrieved test image with shape {image.shape}")
        return image
