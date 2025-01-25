from typing import Type

import pytest

from inputs.base.loop import LoopInput


def get_all_inputs_classes():
    import importlib
    import inspect
    import os

    plugins_dir = os.path.join("src", "inputs", "plugins")
    plugin_files = [f[:-3] for f in os.listdir(plugins_dir) if f.endswith(".py")]

    inputs_classes = []
    for plugin in plugin_files:
        module = importlib.import_module(f"inputs.plugins.{plugin}")
        for name, obj in inspect.getmembers(module):
            if inspect.isclass(obj) and issubclass(obj, LoopInput) and obj != LoopInput:
                inputs_classes.append(obj)
    return inputs_classes


@pytest.mark.parametrize("input_class", get_all_inputs_classes())
def test_init_signature(input_class: Type[LoopInput]):
    # Verify __init__ signature matches base class
    base_params = set(LoopInput.__init__.__annotations__.keys())
    impl_params = set(input_class.__init__.__annotations__.keys())
    assert (
        base_params == impl_params
    ), f"{input_class.__name__} __init__ signature mismatch"


@pytest.mark.parametrize("input_class", get_all_inputs_classes())
def test__poll_to_text_signature(input_class: Type[LoopInput]):
    # Verify _poll method signature matches base class
    base_params = set(LoopInput._poll.__annotations__.keys())
    impl_params = set(input_class._poll.__annotations__.keys())
    assert (
        base_params == impl_params
    ), f"{input_class.__name__} _poll signature mismatch"


@pytest.mark.parametrize("input_class", get_all_inputs_classes())
def test__listen_loop_to_text_signature(input_class: Type[LoopInput]):
    # Verify _listen_loop method signature matches base class
    base_params = set(LoopInput._listen_loop.__annotations__.keys())
    impl_params = set(input_class._listen_loop.__annotations__.keys())
    assert (
        base_params == impl_params
    ), f"{input_class.__name__} _listen_loop signature mismatch"


@pytest.mark.parametrize("input_class", get_all_inputs_classes())
def test__raw_to_text_signature(input_class: Type[LoopInput]):
    # Verify _raw_to_text method signature matches base class
    base_params = set(LoopInput._raw_to_text.__annotations__.keys())
    impl_params = set(input_class._raw_to_text.__annotations__.keys())
    assert (
        base_params == impl_params
    ), f"{input_class.__name__} _raw_to_text signature mismatch"


@pytest.mark.parametrize("input_class", get_all_inputs_classes())
def test_raw_to_text_signature(input_class: Type[LoopInput]):
    # Verify _raw_to_text method signature matches base class
    base_params = set(LoopInput.raw_to_text.__annotations__.keys())
    impl_params = set(input_class.raw_to_text.__annotations__.keys())
    assert (
        base_params == impl_params
    ), f"{input_class.__name__} raw_to_text signature mismatch"


@pytest.mark.parametrize("input_class", get_all_inputs_classes())
def test_formatted_latest_buffer_signature(input_class: Type[LoopInput]):
    # Verify formatted_latest_buffer method signature matches base class
    base_params = set(LoopInput.formatted_latest_buffer.__annotations__.keys())
    impl_params = set(input_class.formatted_latest_buffer.__annotations__.keys())
    assert (
        base_params == impl_params
    ), f"{input_class.__name__} formatted_latest_buffer signature mismatch"


@pytest.mark.parametrize("input_class", get_all_inputs_classes())
def test_listen_signature(input_class: Type[LoopInput]):
    # Verify listen method signature matches base class
    base_params = set(LoopInput.listen.__annotations__.keys())
    impl_params = set(input_class.listen.__annotations__.keys())
    assert (
        base_params == impl_params
    ), f"{input_class.__name__} listen signature mismatch"
