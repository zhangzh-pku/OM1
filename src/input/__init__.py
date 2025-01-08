import os
import importlib
import inspect
import typing as T

from input.base import AgentInput


def load_input(input_name: str) -> T.Type[AgentInput]:
    """
    Load an input plugin from the plugins directory.

    Args:
        input_name: The name of the input plugin class to load.

    Returns:
        An instance of the input plugin.
    """
    # Get all files in plugins directory
    plugins_dir = os.path.join(os.path.dirname(__file__), "plugins")
    plugin_files = [f[:-3] for f in os.listdir(plugins_dir) if f.endswith(".py")]

    # Import all plugin modules and find AgentInput subclasses
    input_classes = {}
    for plugin in plugin_files:
        module = importlib.import_module(f"input.plugins.{plugin}")
        for name, obj in inspect.getmembers(module):
            if (
                inspect.isclass(obj)
                and issubclass(obj, AgentInput)
                and obj != AgentInput
            ):
                input_classes[name] = obj

    # Find requested input class
    if input_name not in input_classes:
        raise ValueError(f"Input type {input_name} not found")

    return input_classes[input_name]
