from actions.base import (
    ActionConfig,
    ActionConnector,
    ActionImplementation,
    AgentAction,
    Interface,
)


def test_action_config():
    """Test action configuration initialization."""
    config = ActionConfig(
        name="test_action",
        implementation="test_impl",
        interface="test_interface",
        connector="test_connector",
    )
    assert config.name == "test_action"
    assert config.implementation == "test_impl"
    assert config.interface == "test_interface"
    assert config.connector == "test_connector"
    assert config.config == {}


def test_action_config_with_extra_params():
    """Test action configuration with additional parameters."""
    config = ActionConfig(
        name="test_action",
        implementation="test_impl",
        interface="test_interface",
        connector="test_connector",
        config={"param1": "value1", "param2": 42},
    )
    assert config.config["param1"] == "value1"
    assert config.config["param2"] == 42


def test_agent_action_initialization():
    """Test AgentAction initialization with config."""

    class TestInterface(Interface[str, str]):
        pass

    class TestImplementation(ActionImplementation):
        async def execute(self, input_protocol):
            pass

    class TestConnector(ActionConnector):
        async def connect(self, input_protocol):
            pass

    config = ActionConfig(
        name="test_action",
        implementation="test_impl",
        interface="test_interface",
        connector="test_connector",
    )

    action = AgentAction(
        config=config,
        interface=TestInterface,
        implementation=TestImplementation(),
        connector=TestConnector(),
    )

    assert action.name == "test_action"
    assert action.config == config
    assert action.interface == TestInterface
