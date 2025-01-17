import asyncio
import logging

from fuser import Fuser
from input.orchestrator import InputOrchestrator
from modules.orchestrator import ModuleOrchestrator
from runtime.config import RuntimeConfig
from providers.sleep_ticker_provider import SleepTickerProvider


class CortexRuntime:
    """
    The CortexRuntime is the main entry point for the OM2 agent.
    It is responsible for running the agent, orchestrating communication between the memory, fuser, actions, and managing the inputs and outputs.
    """

    config: RuntimeConfig
    fuser: Fuser
    module_orchestrator: ModuleOrchestrator
    sleep_ticker_provider: SleepTickerProvider

    def __init__(self, config: RuntimeConfig):
        self.config = config
        self.fuser = Fuser(config)
        self.module_orchestrator = ModuleOrchestrator(config)
        self.sleep_ticker_provider = SleepTickerProvider()

    async def run(self) -> None:
        input_listener_task = await self._start_input_listeners()
        cortex_loop_task = asyncio.create_task(self._run_cortex_loop())
        await asyncio.gather(input_listener_task, cortex_loop_task)

    async def _start_input_listeners(self) -> asyncio.Task:
        input_orchestrator = InputOrchestrator(self.config.agent_inputs)
        input_listener_task = asyncio.create_task(input_orchestrator.listen())
        return input_listener_task

    async def _run_cortex_loop(self) -> None:
        while True:
            if not self.sleep_ticker_provider.skip_sleep:
                await self.sleep_ticker_provider.sleep(1 / self.config.hertz)
            await self._tick()
            self.sleep_ticker_provider.skip_sleep = False

    async def _tick(self) -> None:
        finished_promises, _ = await self.module_orchestrator.flush_promises()
        prompt = self.fuser.fuse(self.config.agent_inputs, finished_promises)
        if prompt is None:
            logging.warning("No prompt to fuse")
            return
        output = await self.config.cortex_llm.ask(prompt)
        if output is None:
            logging.warning("No output from LLM")
            return

        logging.debug("I'm thinking... ", output)
        await self.module_orchestrator.promise(output.commands)
