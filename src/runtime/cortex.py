import asyncio
import logging

from fuser import Fuser
from inputs.orchestrator import InputOrchestrator
from actions.orchestrator import ActionOrchestrator
from simulators.orchestrator import SimulatorOrchestrator

from runtime.config import RuntimeConfig
from providers.sleep_ticker_provider import SleepTickerProvider


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
        # collect all the latest inputs
        finished_promises, _ = await self.action_orchestrator.flush_promises()
        
        # combine those inputs into a suitable prompt
        [prompt, full_inputs_with_ts]= self.fuser.fuse(self.config.agent_inputs, finished_promises)
        if prompt is None:
            logging.warning("No prompt to fuse")
            return
        
        # if there is a prompt, send to the AIs
        payload = await self.config.cortex_llm.ask(prompt, full_inputs_with_ts)
        if payload is None:
            logging.warning("No output from LLM")
            return

        logging.debug(f"LLM returned this: {payload}")
        
        # take all the commands and send them to the correct actions to 
        # trigger actions
        await self.simulator_orchestrator.promise(payload)

        #logging.debug(f"Pushing output to execution: {output}")
        await self.action_orchestrator.promise(payload.commands.commands)
