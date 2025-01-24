import json
import os
from dataclasses import dataclass

from actions import load_action
from actions.base import AgentAction
from inputs import load_input
from inputs.base import AgentInput
from llm import LLM, LLMConfig, load_llm
from llm.output_model import CortexOutputModel
from simulators import load_simulator
from simulators.base import Simulator


@dataclass
class RuntimeConfig:
    hertz: float
    name: str
    system_prompt: str
    agent_inputs: list[AgentInput]
    cortex_llm: LLM[CortexOutputModel]
    agent_actions: list[AgentAction]
    simulators: list[Simulator]


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
        "simulators": [
            load_simulator(simulator["type"])()
            for simulator in raw_config["simulators"]
        ],
        "cortex_llm": load_llm(raw_config["cortex_llm"]["type"])(
            config=LLMConfig(**raw_config["cortex_llm"].get("config", {})),
            output_model=CortexOutputModel,
        ),
        "agent_actions": [
            load_action(action) for action in raw_config["agent_actions"]
        ],
    }
    return RuntimeConfig(**parsed_config)
