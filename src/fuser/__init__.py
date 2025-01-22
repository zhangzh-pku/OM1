import typing as T

from inputs.base import AgentInput
from actions import describe_action
from runtime.config import RuntimeConfig
import time

import logging

class Fuser:
    """
    Fuses inputs into a single output
    """

    def __init__(self, config: RuntimeConfig):
        self.config = config

    def fuse(
        self, inputs: list[AgentInput], finished_promises: list[T.Any]
    ) -> str | None:
        """Combine all inputs, memories, and configurations into a single prompt"""
        system_prompt = self.config.system_prompt

        # for input in inputs:
        #     message = input.formatted_latest_buffer() 
        #     logging.info(f"InputMessage: {message}")

        input_strings = []
        input_strings_full = []
        for input in inputs:
            input_raw = input.formatted_latest_buffer()
            # remove the timestamp at the front
            input_strings_full.append(input_raw)
            if input_raw is not None and "::" in input_raw:
                input_raw = input_raw.split("::")[-1]
            input_strings.append(input_raw)

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
        fused_prompt = f"{time.time():.3f}::{system_prompt}\n\n{inputs_fused}\n\nAVAILABLE MODULES:\n{actions_fused}\n\n{question_prompt}"
        return [fused_prompt, input_strings_full]
