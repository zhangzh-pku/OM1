import asyncio
import logging

from actions.orchestrator import ActionOrchestrator
from fuser import Fuser
from inputs.orchestrator import InputOrchestrator
from providers.sleep_ticker_provider import SleepTickerProvider
from runtime.config import RuntimeConfig
from simulators.orchestrator import SimulatorOrchestrator


class CortexRuntime:
    """
    The CortexRuntime is the main entry point for the omOS agent.
    It is responsible for running the agent, orchestrating communication
    between the memory, fuser, actions, and managing the inputs and outputs.
    """

    config: RuntimeConfig
    fuser: Fuser
    action_orchestrator: ActionOrchestrator
    simulator_orchestrator: SimulatorOrchestrator
    sleep_ticker_provider: SleepTickerProvider

    def __init__(self, config: RuntimeConfig):
        self.config = config
        self.fuser = Fuser(config)
        self.action_orchestrator = ActionOrchestrator(config)
        self.simulator_orchestrator = SimulatorOrchestrator(config)
        self.sleep_ticker_provider = SleepTickerProvider()

    async def run(self) -> None:
        simulator_start = self._start_simulator_task()
        input_listener_task = await self._start_input_listeners()
        cortex_loop_task = asyncio.create_task(self._run_cortex_loop())
        await asyncio.gather(simulator_start, input_listener_task, cortex_loop_task)

    async def _start_input_listeners(self) -> asyncio.Task:
        input_orchestrator = InputOrchestrator(self.config.agent_inputs)
        input_listener_task = asyncio.create_task(input_orchestrator.listen())
        return input_listener_task

    async def _start_simulator_task(self) -> asyncio.Task:
        simulator_task = asyncio.create_task(self.simulator_orchestrator.start())
        return simulator_task

    async def _run_cortex_loop(self) -> None:
        while True:
            if not self.sleep_ticker_provider.skip_sleep:
                await self.sleep_ticker_provider.sleep(1 / self.config.hertz)
            await self._tick()
            self.sleep_ticker_provider.skip_sleep = False

    async def _tick(self) -> None:
        # collect all the latest inputs
        finished_promises, _ = await self.action_orchestrator.flush_promises()

        # combine those inputs into a suitable prompt
        prompt = self.fuser.fuse(self.config.agent_inputs, finished_promises)
        if prompt is None:
            logging.warning("No prompt to fuse")
            return

        # if there is a prompt, send to the AIs
        output = await self.config.cortex_llm.ask(prompt)
        if output is None:
            logging.warning("No output from LLM")
            return

        # Trigger the simulators
        await self.simulator_orchestrator.promise(output.commands)
        # Trigger the actions
        await self.action_orchestrator.promise(output.commands)
