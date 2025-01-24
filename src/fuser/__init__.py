import logging
import time
import typing as T

from actions import describe_action
from inputs.base import AgentInput
from providers.io_provider import IOProvider
from runtime.config import RuntimeConfig


class Fuser:
    """
    Fuses inputs into a single output
    """

    def __init__(self, config: RuntimeConfig):
        self.config = config
        self.io_provider = IOProvider()

    def fuse(self, inputs: list[AgentInput], finished_promises: list[T.Any]) -> str:
        # Record the timestamp of the input
        self.io_provider.fuser_start_time = time.time()

        """Combine all inputs, memories, and configurations into a single prompt"""
        system_prompt = self.config.system_prompt

        input_strings = [input.formatted_latest_buffer() for input in inputs]
        logging.debug(f"InputMessageArray: {input_strings}")

        inputs_fused = " ".join([s for s in input_strings if s is not None])
        # descriptions of various possible actions
        actions_fused = "\n\n\n".join(
            [describe_action(action.name) for action in self.config.agent_actions]
        )
        question_prompt = "What will you do? Command: "
        # this is the final prompt:
        # (1) a (typically) fixed overall system prompt with the agents, name, rules, and examples
        # (2) all the inputs (vision, sound, etc.)
        # (3) a (typically) fixed list of available actions
        # (4) a (typically) fixed system prompt requesting commands to be generated
        fused_prompt = f"{system_prompt}\n\n{inputs_fused}\n\nAVAILABLE MODULES:\n{actions_fused}\n\n{question_prompt}"

        # Record the timestamp of the output
        self.io_provider.fuser_end_time = time.time()

        return fused_prompt
