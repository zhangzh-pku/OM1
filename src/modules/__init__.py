from enum import Enum
import typing as T
import importlib

from modules.base import Interface, Module, ModuleImpl, ModuleMutation


def describe_module(module_name: str) -> str:
    interface = None
    module = importlib.import_module(f"modules.{module_name}.interface")
    for _, obj in module.__dict__.items():
        if isinstance(obj, type) and issubclass(obj, Interface) and obj != Interface:
            interface = obj
    if interface is None:
        raise ValueError(f"No interface found for module {module_name}")
    # Get docstring from interface class
    doc = interface.__doc__ or ""

    # Build type hint descriptions
    hints = {}
    input_interface = T.get_type_hints(interface)["input"]
    for field_name, field_type in T.get_type_hints(input_interface).items():
        if hasattr(field_type, "__origin__") and field_type.__origin__ == type:
            # Handle generic types
            hints[field_name] = str(field_type)
        elif isinstance(field_type, type) and issubclass(field_type, Enum):
            # Handle enums by listing allowed values
            values = [f"'{v.value}'" for v in field_type]
            hints[field_name] = (
                f"{field_type.__name__} - Allowed values: {', '.join(values)}"
            )
        else:
            hints[field_name] = str(field_type)

    # Format the full docstring
    type_hints = "\n".join(f"    {name}: {desc}" for name, desc in hints.items())
    return f"command: {module_name}\n{doc}\nArguments:\n{type_hints}"


def load_module(module_config: dict[str, str]) -> Module:
    interface = None
    module = importlib.import_module(f"modules.{module_config['name']}.interface")
    for _, obj in module.__dict__.items():
        if isinstance(obj, type) and issubclass(obj, Interface) and obj != Interface:
            interface = obj
    if interface is None:
        raise ValueError(f"No interface found for module {module_config['name']}")
    impl = importlib.import_module(
        f"modules.{module_config['name']}.impl.{module_config['impl']}"
    )
    mutation = importlib.import_module(
        f"modules.{module_config['name']}.mutation.{module_config['mutation']}"
    )
    impl_class = None
    mutation_class = None
    for _, obj in impl.__dict__.items():
        if isinstance(obj, type) and issubclass(obj, ModuleImpl):
            impl_class = obj
    for _, obj in mutation.__dict__.items():
        if isinstance(obj, type) and issubclass(obj, ModuleMutation):
            mutation_class = obj
    if impl_class is None:
        raise ValueError(
            f"No implementation found for module {module_config['name']} impl {module_config['impl']}"
        )
    if mutation_class is None:
        raise ValueError(
            f"No mutation found for module {module_config['name']} mutation {module_config['mutation']}"
        )
    return Module(
        name=module_config["name"],
        interface=interface,
        impl=impl_class(),
        mutation=mutation_class(),
    )
