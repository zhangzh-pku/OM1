import threading
from typing import Dict, Optional

from .singleton import singleton

@singleton
class IOProvider:

    def __init__(self):
        self._lock: threading.Lock = threading.Lock()
        self._inputs: Dict[str, str] = {}
        self._input_timestamps: Dict[str, float] = {}
        self._fuser_start_time: Optional[float] = None
        self._fuser_end_time: Optional[float] = None
        self._llm_prompt: Optional[str] = None
        self._llm_start_time: Optional[float] = None
        self._llm_end_time: Optional[float] = None

    # I want to combine timestamp to input if it is not None
    @property
    def inputs(self) -> Dict[str, Dict[str, str]]:
        with self._lock:
            result = {}
            for name, value in self._inputs.items():
                timestamp = self._input_timestamps.get(name)
                if timestamp is not None:
                    result[name] = {
                        "input": value,
                        "timestamp": timestamp
                    }
                else:
                    result[name] = value
            return result

    def add_input(self, key: str, value: str, timestamp: Optional[float]) -> None:
        with self._lock:
            self._inputs[key] = value
            if timestamp is not None:
                self._input_timestamps[key] = timestamp

    def remove_input(self, key: str) -> None:
        with self._lock:
            self._inputs.pop(key, None)
            self._input_timestamps.pop(key, None)

    def add_input_timestamp(self, key: str, timestamp: float) -> None:
        with self._lock:
            self._input_timestamps[key] = timestamp

    def get_input_timestamp(self, key: str) -> Optional[float]:
        with self._lock:
            return self._input_timestamps.get(key)

    @property
    def fuser_start_time(self) -> Optional[float]:
        with self._lock:
            return self._fuser_start_time

    @fuser_start_time.setter
    def fuser_start_time(self, value: Optional[float]) -> None:
        with self._lock:
            self._fuser_start_time = value

    def set_fuser_start_time(self, value: Optional[float]) -> None:
        with self._lock:
            self._fuser_start_time = value

    @property
    def fuser_end_time(self) -> Optional[float]:
        with self._lock:
            return self._fuser_end_time

    @fuser_end_time.setter
    def fuser_end_time(self, value: Optional[float]) -> None:
        with self._lock:
            self._fuser_end_time = value

    def set_fuser_end_time(self, value: Optional[float]) -> None:
        with self._lock:
            self._fuser_end_time = value

    @property
    def llm_prompt(self) -> Optional[str]:
        with self._lock:
            return self._llm_prompt

    @llm_prompt.setter
    def llm_prompt(self, value: Optional[str]) -> None:
        with self._lock:
            self._llm_prompt = value

    def set_llm_prompt(self, value: Optional[str]) -> None:
        with self._lock:
            self._llm_prompt = value

    def clear_llm_prompt(self) -> None:
        with self._lock:
            self._llm_prompt = None

    @property
    def llm_start_time(self) -> Optional[float]:
        with self._lock:
            return self._llm_start_time

    @llm_start_time.setter
    def llm_start_time(self, value: Optional[float]) -> None:
        with self._lock:
            self._llm_start_time = value

    def set_llm_start_time(self, value: Optional[float]) -> None:
        with self._lock:
            self._llm_start_time = value

    @property
    def llm_end_time(self) -> Optional[float]:
        with self._lock:
            return self._llm_end_time

    @llm_end_time.setter
    def llm_end_time(self, value: Optional[float]) -> None:
        with self._lock:
            self._llm_end_time = value