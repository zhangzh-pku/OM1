import threading
import asyncio
from typing import Optional

from .singleton import singleton

@singleton
class SleepTickerProvider:
    def __init__(self):
        self._lock = threading.Lock()
        self._skip_sleep = False
        self._current_sleep_task: Optional[asyncio.Task] = None

    @property
    def skip_sleep(self) -> bool:
        with self._lock:
            return self._skip_sleep

    @skip_sleep.setter
    def skip_sleep(self, value: bool) -> None:
        with self._lock:
            self._skip_sleep = value
            if value and self._current_sleep_task:
                self._current_sleep_task.cancel()

    async def sleep(self, duration: float) -> None:
        try:
            self._current_sleep_task = asyncio.create_task(asyncio.sleep(duration))
            await self._current_sleep_task
        except asyncio.CancelledError:
            pass
        finally:
            self._current_sleep_task = None