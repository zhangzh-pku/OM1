import typing as T

from input.base import AgentInput
from modules import describe_module
from runtime.config import RuntimeConfig

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

        input_strings = [input.formatted_latest_buffer() for input in inputs]
        logging.info(f"InputMessageArray: {input_strings}")
        inputs_fused = " ".join([s for s in input_strings if s is not None])
        modules_fused = "\n\n\n".join(
            [describe_module(module.name) for module in self.config.modules]
        )
        question_prompt = "What will you do? Command: "
        fused_prompt = f"{system_prompt}\n\n{inputs_fused}\n\nAVAILABLE MODULES:\n{modules_fused}\n\n{question_prompt}"
        return fused_prompt
