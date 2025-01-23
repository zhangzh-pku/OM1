import asyncio
import logging
import typing as T

from llm.output_model import Command
from simulators.base import Simulator
from runtime.config import RuntimeConfig


class SimulatorOrchestrator:
    """
    Manages data flow to one or more simulators.
    Note: It is important that the simulators do not block the event loop.
    """

    promise_queue: list[asyncio.Task[T.Any]]
    _config: RuntimeConfig

    def __init__(self, config: RuntimeConfig):
        self._config = config
        self.promise_queue = []

    async def flush_promises(self) -> tuple[list[T.Any], list[asyncio.Task[T.Any]]]:
        """
        Flushes the promise queue and returns the completed promises
        and the pending promises.
        """
        done_promises = []
        for promise in self.promise_queue:
            if promise.done():
                await promise
                done_promises.append(promise)
        self.promise_queue = [p for p in self.promise_queue if p not in done_promises]
        return done_promises, self.promise_queue

    async def promise(self, commands: T.List[Command]) -> None:
        # loop through simulators and send each one of them
        # the current LLM_full
        for simulator in self._config.simulators:
            simulator_response = asyncio.create_task(self._promise_simulator(simulator, commands))
            self.promise_queue.append(simulator_response)

    async def _promise_simulator(self, simulator: Simulator, commands: T.List[Command]) -> T.Any:
        logging.debug(
            f"Calling simulator {simulator.name} with commands {commands}"
        )
        simulator.run(commands)
        return None
