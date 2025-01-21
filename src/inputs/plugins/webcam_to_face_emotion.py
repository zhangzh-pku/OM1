import asyncio
import random
import logging
import time

from typing import Dict, Optional

from inputs.base.loop import LoopInput

import cv2
from deepface import DeepFace

"""
Code example is from:
https://github.com/manish-9245/Facial-Emotion-Recognition-using-OpenCV-and-Deepface
Thank you @manish-9245
"""

class FaceEmotionCapture(LoopInput[cv2.typing.MatLike]):
    """
    Uses a webcam and returns a label of the person's emotions
    """

    def __init__(self):
        # Load face cascade classifier
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        # Start capturing video
        self.cap = cv2.VideoCapture(0)
        self.emotion = ""
        self.messages: list[str] = []

    async def _poll(self) -> cv2.typing.MatLike:
        await asyncio.sleep(0.2)
        
        # Capture a frame every 200 ms
        ret, frame = self.cap.read()

        return frame

    async def _raw_to_text(self, raw_input: cv2.typing.MatLike) -> str:
        
        frame = raw_input

        # Convert frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Convert grayscale frame to RGB format
        rgb_frame = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2RGB)

        # Detect faces in the frame
        faces = self.face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        for (x, y, w, h) in faces:
            # Extract the face ROI (Region of Interest)
            face_roi = rgb_frame[y:y + h, x:x + w]

            # Perform emotion analysis on the face ROI
            result = DeepFace.analyze(face_roi, actions=['emotion'], enforce_detection=False)

            # Determine the dominant emotion
            self.emotion = result[0]['dominant_emotion']

            # # Draw rectangle around face and label with predicted emotion
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # cv2.putText(frame, self.emotion, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        # # Display the resulting frame
        # cv2.imshow('Real-time Emotion Detection', frame)

        message = f"{time.time():.3f}::I see a person. Their emotion is {self.emotion}."
        logging.debug(f"FaceEmotionCapture: {message}")
        
        return message
        
    async def raw_to_text(self, raw_input):
        """
        Convert raw input to processed text and manage buffer.

        Parameters
        ----------
        raw_input : Optional[str]
            Raw input to be processed
        """
        text = await self._raw_to_text(raw_input)
        if text is None:
            if len(self.messages) == 0:
                return None
            # else:
            #     # Skip sleep if there's already a message in the buffer
            #     self.global_sleep_ticker_provider.skip_sleep = True

        if text is not None:
            if len(self.messages) == 0:
                self.messages.append(text)
            # else:
            # here is where you can implementationement joining older messages
            #     self.buffer[-1] = f"{self.buffer[-1]} {text}"

    def formatted_latest_buffer(self) -> Optional[str]:
        """
        Format and clear the latest buffer contents.

        Returns
        -------
        Optional[str]
            Formatted string of buffer contents or None if buffer is empty
        """
        if len(self.messages) == 0:
            return None

        message = self.messages[-1]
        part = message.split("::")
        ts = part[0]
        ms = part[-1]

        result = f"""{ts}::
        {self.__class__.__name__} INPUT
        // START
        {ms}
        // END
        """

        self.messages = []
        return result
