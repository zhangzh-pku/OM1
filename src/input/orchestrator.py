import asyncio
import typing as T

from input.base import AgentInput


class InputOrchestrator:
    """
    Manages data flow for the inputs
    """

    inputs: list[AgentInput]

    def __init__(self, inputs: list[AgentInput]):
        self.inputs = inputs

    async def listen(self) -> None:
        input_tasks = [
            asyncio.create_task(self._listen_to_input(input)) for input in self.inputs
        ]
        await asyncio.gather(*input_tasks)

    async def _listen_to_input(self, input: AgentInput) -> None:
        async for event in input.listen():
            await input.raw_to_text(event)
