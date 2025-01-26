import asyncio

from inputs.base import AgentInput


class InputOrchestrator:
    """
    Manages and coordinates multiple input sources.

    Handles concurrent processing of multiple AgentInput instances,
    orchestrating their data flows.

    Parameters
    ----------
    inputs : list[AgentInput]
        List of input sources to manage
    """

    inputs: list[AgentInput]

    def __init__(self, inputs: list[AgentInput]):
        """
        Initialize InputOrchestrator instance with input sources.
        """
        self.inputs = inputs

    async def listen(self) -> None:
        """
        Start listening to all input sources concurrently.

        Creates and manages async tasks for each input source.
        """
        input_tasks = [
            asyncio.create_task(self._listen_to_input(input)) for input in self.inputs
        ]
        await asyncio.gather(*input_tasks)

    async def _listen_to_input(self, input: AgentInput) -> None:
        """
        Process events from a single input source.

        Parameters
        ----------
        input : AgentInput
            Input source to listen to
        """
        async for event in input.listen():
            await input.raw_to_text(event)
