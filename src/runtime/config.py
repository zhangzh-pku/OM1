import os
import json
from dataclasses import dataclass

from input import load_input
from input.base import AgentInput
from llm import LLM, load_llm
from llm.output_model import CortexOutputModel
from modules.base import Module
from modules import load_module


@dataclass
class RuntimeConfig:
    hertz: float
    name: str
    system_prompt: str
    agent_inputs: list[AgentInput]
    cortex_llm: LLM[CortexOutputModel]
    modules: list[Module]


def load_config(config_name: str) -> RuntimeConfig:
    config_path = os.path.join(
        os.path.dirname(__file__), "../../config", config_name + ".json"
    )
    with open(config_path, "r") as f:
        raw_config = json.load(f)

    parsed_config = {
        **raw_config,
        "agent_inputs": [
            load_input(input["type"])() for input in raw_config["agent_inputs"]
        ],
        "cortex_llm": load_llm(raw_config["cortex_llm"]["type"])(
            output_model=CortexOutputModel
        ),
        "modules": [load_module(module) for module in raw_config["modules"]],
    }
    return RuntimeConfig(**parsed_config)
