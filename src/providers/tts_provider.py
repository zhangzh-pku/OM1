import logging
import threading
import time
from typing import Callable, Optional

import pyaudio
from om1_speech import AudioOutputStream

from .singleton import singleton


@singleton
class TTSProvider:
    """
    Text-to-Speech Provider that manages an audio output stream.

    A singleton class that handles text-to-speech conversion and audio output
    through a dedicated thread.

    Parameters
    ----------
    url : str
        The URL endpoint for the TTS service
    """

    def __init__(self, url: str):
        """
        Initialize the TTS provider with given URL.

        Parameters
        ----------
        url : str
            The URL endpoint for the TTS service
        """
        self.running: bool = False

        p = pyaudio.PyAudio()
        SPEAKER = p.get_default_output_device_info()
        logging.info(
            f"SYSTEM DEFAULT SPEAKER: idx:{SPEAKER["index"]} name:{SPEAKER["name"]}"
        )

        self.audio_stream: AudioOutputStream = AudioOutputStream(
            url=url, device=int(SPEAKER["index"])
        )
        self._thread: Optional[threading.Thread] = None

    def register_tts_state_callback(self, tts_state_callback: Optional[Callable]):
        """
        Register a callback for TTS state changes.

        Parameters
        ----------
        tts_state_callback : callable
            The callback function to receive TTS state changes.
        """
        self.audio_stream.set_tts_state_callback(tts_state_callback)

    def add_pending_message(self, text: str):
        """
        Add text to the pending queue for TTS processing.

        Parameters
        ----------
        text : str
            Text to be converted to speech
        """
        logging.info(f"audio_stream: {text}")
        self.audio_stream.add(text)

    def start(self):
        """
        Start the TTS provider and its audio stream.
        """
        if self.running:
            return

        self.running = True
        self.audio_stream.start()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        """
        Internal method to run the TTS processing loop.
        """
        while self.running:
            try:
                time.sleep(0.1)
            except Exception as e:
                logging.error(f"TTSProvider error: {e}")

    def stop(self):
        """
        Stop the TTS provider and cleanup resources.
        """
        self.running = False
        if self._thread:
            self.audio_stream.stop()
            self._thread.join(timeout=5)
