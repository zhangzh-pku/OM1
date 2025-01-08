import os
import importlib
import inspect
import typing as T

R = T.TypeVar("R")


class LLM(T.Generic[R]):
    """
    LLM interface
    """

    def __init__(self, output_model: T.Type[R]):
        self._output_model = output_model

    async def ask(self, prompt: str) -> R:
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

    # Import all plugin modules and find LLM subclasses
    llm_classes = {}
    for plugin in plugin_files:
        module = importlib.import_module(f"llm.plugins.{plugin}")
        for name, obj in inspect.getmembers(module):
            if inspect.isclass(obj) and issubclass(obj, LLM) and obj != LLM:
                llm_classes[name] = obj

    # Find requested input class
    if llm_name not in llm_classes:
        raise ValueError(f"LLM type {llm_name} not found")

    return llm_classes[llm_name]
