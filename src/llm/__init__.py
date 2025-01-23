import os
import importlib
from dataclasses import dataclass
from pydantic import BaseModel
import inspect
import typing as T

from providers.io_provider import IOProvider

R = T.TypeVar("R")

class LLMConfig(BaseModel):
    base_url: T.Optional[str] = None
    api_key: T.Optional[str] = None


class LLM(T.Generic[R]):
    """
    LLM interface
    """

    def __init__(self, output_model: T.Type[R], config: T.Optional[LLMConfig]):
        # Set up the LLM configuration
        self._conifg = config

        # Set up the output model
        self._output_model = output_model

        # Set up the IO provider
        self.io_provider = IOProvider()

    async def ask(self, prompt: str, inputs: list[str]) -> R:
        """Ask the LLM a question"""
        raise NotImplementedError


def load_llm(llm_name: str) -> T.Type[LLM]:
    """
    Load an LLM plugin from the plugins directory.

    Args:
        input_name: The name of the input plugin class to load.

    Returns:
        An instance of the input plugin.
    """
    # Get all files in plugins directory
    plugins_dir = os.path.join(os.path.dirname(__file__), "plugins")
    plugin_files = [f[:-3] for f in os.listdir(plugins_dir) if f.endswith(".py")]

    # Import all plugin actions and find LLM subclasses
    llm_classes = {}
    for plugin in plugin_files:
        action = importlib.import_module(f"llm.plugins.{plugin}")
        for name, obj in inspect.getmembers(action):
            if inspect.isclass(obj) and issubclass(obj, LLM) and obj != LLM:
                llm_classes[name] = obj

    # Find requested input class
    if llm_name not in llm_classes:
        raise ValueError(f"LLM type {llm_name} not found")

    return llm_classes[llm_name]
