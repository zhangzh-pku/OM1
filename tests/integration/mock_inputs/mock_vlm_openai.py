import logging
import time
import cv2
import base64

from inputs.base import SensorConfig
from inputs.plugins.vlm_openai import VLMOpenAI
from tests.integration.mock_inputs.mock_image_provider import get_next_opencv_image


class MockVLM_OpenAI(VLMOpenAI):
    """
    Mock implementation of VLM_OpenAI that uses the central image provider.

    This class overrides only the camera input functionality to get images from
    the mock image provider, while maintaining all the real object detection logic.
    """

    def __init__(self, config: SensorConfig = SensorConfig()):
        """
        Initialize with the real VLM implementation but without opening camera.

        Parameters
        ----------
        config : SensorConfig, optional
            Configuration for the sensor
        """
        # Initialize using the real VLM_OpenAI implementation
        super().__init__(config)

        # Override the descriptor to indicate this is a mock
        self.descriptor_for_LLM = "MOCK Vision INPUT (OpenAI Test)"

        # Replace the real video stream frame callback with our mock version
        # This is the key part - we intercept the video stream and feed it mock images
        if hasattr(self.vlm, 'video_stream'):
            original_callback = self.vlm.video_stream.frame_callbacks
            self.vlm.video_stream.frame_callbacks = self._mock_frame_callbacks
            self.original_callback = original_callback
            
            # Store the last processed time to rate-limit our mock frames
            self.last_processed_time = 0
            
            logging.info("MockVLM_OpenAI: Replaced video stream callback with mock version")
        else:
            logging.error("MockVLM_OpenAI: Could not find video_stream in VLM provider")
    
    async def _mock_frame_callbacks(self, _):
        """
        Mock callback that processes frames from our mock image provider.
        
        This replaces the original frame callback in the video stream with one
        that processes mock images instead of camera frames.
        
        Parameters
        ----------
        _ : str
            Original base64 frame (ignored)
        """
        # Rate limit to avoid overwhelming the API
        current_time = time.time()
        if current_time - self.last_processed_time < 1.0:  # One second minimum between frames
            return
            
        self.last_processed_time = current_time
        
        # Get a mock image
        image = get_next_opencv_image()
        if image is None:
            return
            
        logging.info(f"MockVLM_OpenAI: Processing mock image with shape {image.shape}")
        
        # Convert image to base64 string
        _, buffer = cv2.imencode('.jpg', image)
        base64_image = base64.b64encode(buffer).decode('utf-8')
        
        # Process the image using the original callback
        await self.original_callback(base64_image)
