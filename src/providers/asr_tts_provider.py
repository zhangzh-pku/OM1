import threading

from singleton import singleton

@singleton
class ASRTTSProvider:
    """
    ASRTTSProvider is a singleton class that manages the Text-to-Speech (TTS) state.
    Thread-safe implementation ensures consistent state across multiple threads.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._is_tts_active = False

    @property
    def is_tts_active(self) -> bool:
        """
        Get the current TTS active state.

        Returns:
            bool: True if TTS is active, False otherwise
        """
        with self._lock:
            return self._is_tts_active

    @is_tts_active.setter
    def is_tts_active(self, value: bool) -> None:
        """
        Set the TTS active state.

        Args:
            value (bool): The new TTS state to set
        """
        with self._lock:
            self._is_tts_active = value

    def start_tts(self) -> None:
        """Activate the TTS system."""
        self._is_tts_active = True

    def stop_tts(self) -> None:
        """Deactivate the TTS system."""
        self._is_tts_active = False
