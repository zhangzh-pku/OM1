import asyncio
import logging
import typing as T
from llm.output_model import Command
from actions.base import AgentAction
from runtime.config import RuntimeConfig


class ActionOrchestrator:
    """
    Manages data flow for the actions.

    Note: It is very important that the actions do not block the event loop.
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
        # loop through commands and send the correct 
        # command to the correct action
        for command in commands:
            logging.info(f"Sending command: {command}")
            action = next(
                (m for m in self._config.agent_actions if m.name == command.name), None
            )
            if action is None:
                logging.warning(
                    f"Attempted to call non-existant action: {command.name}"
                )
                continue
            action_response = asyncio.create_task(self._promise_action(action, command))
            self.promise_queue.append(action_response)

    async def _promise_action(self, action: AgentAction, command: Command) -> T.Any:
        logging.debug(
            f"Calling action {action.name} with arguments {command.arguments}"
        )
        input_interface = T.get_type_hints(action.interface)["input"](
            **{arg.name: arg.value for arg in command.arguments}
        )
        action_response = await action.implementation.execute(input_interface)
        logging.debug(f"Action {action.name} returned {action_response}")
        await action.connector.connect(action_response)
        return action_response
