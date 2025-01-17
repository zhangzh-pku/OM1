import asyncio
import random
from PIL import Image
import logging

from input.base.loop import LoopInput

import cv2
from deepface import DeepFace

"""
Code example is from:
https://github.com/manish-9245/Facial-Emotion-Recognition-using-OpenCV-and-Deepface
Thank you @manish-9245
"""

class FaceEmotionCapture(LoopInput[Image.Image]):
    """
    Takes data from a webcam and returns a label of the person's emotions
    """

    def __init__(self):
        # Load face cascade classifier
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        # Start capturing video
        self.cap = cv2.VideoCapture(0)
        self.emotion = ""

    async def _poll(self) -> Image.Image:
        await asyncio.sleep(0.2)
        
        # Capture a frame every 200 ms
        ret, frame = self.cap.read()

        return frame

    async def _raw_to_text(self, raw_input: Image.Image) -> str:
        
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

            # Draw rectangle around face and label with predicted emotion
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, self.emotion, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        # Display the resulting frame
        cv2.imshow('Real-time Emotion Detection', frame)

        message = f"I see a person that is {self.emotion}"

        logging.info(f"FaceEmotionCapture: {self.emotion}")
        
        return message
        
#     # Press 'q' to exit
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Release the capture and close all windows
# cap.release()
# cv2.destroyAllWindows()