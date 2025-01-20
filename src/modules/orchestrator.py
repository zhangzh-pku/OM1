import asyncio
import logging
import typing as T
from llm.output_model import Command
from modules.base import Module
from runtime.config import RuntimeConfig


class ModuleOrchestrator:
    """
    Manages data flow for the modules.

    Note: It is very important that the modules do not block the event loop.
    """

    promise_queue: list[asyncio.Task[T.Any]]
    _config: RuntimeConfig

    def __init__(self, config: RuntimeConfig):
        self._config = config
        self.promise_queue = []

    async def flush_promises(self) -> tuple[list[T.Any], list[asyncio.Task[T.Any]]]:
        """
        Flushes the promise queue and returns the completed promises and the pending promises.
        """
        done_promises = []
        for promise in self.promise_queue:
            if promise.done():
                await promise
                done_promises.append(promise)
        self.promise_queue = [p for p in self.promise_queue if p not in done_promises]
        return done_promises, self.promise_queue

    async def promise(self, commands: list[Command]) -> None:
        for command in commands:
            module = next(
                (m for m in self._config.modules if m.name == command.name), None
            )
            if module is None:
                logging.warning(
                    f"Attempted to call non-existant module: {command.name}"
                )
                continue
            module_response = asyncio.create_task(self._promise_module(module, command))
            self.promise_queue.append(module_response)

    async def _promise_module(self, module: Module, command: Command) -> T.Any:
        logging.debug(
            f"Calling module {module.name} with arguments {command.arguments}"
        )
        input_interface = T.get_type_hints(module.interface)["input"](
            **{arg.name: arg.value for arg in command.arguments}
        )
        module_response = await module.impl.execute(input_interface)
        logging.debug(f"Module {module.name} returned {module_response}")
        await module.connector.connect(module_response)
        return module_response
