# OpenMind OS (omOS)

OpenMind OS is an agent runtime system that enables the creation and execution of digital and physical embodied AI agents with modular capabilities like movement, speech, and perception. A key benefit of using omOS is the ease of deploying consistent digital personas across virtual and physical environments.

## Quick Start

1. Install the Rust python package manager `uv`:

```bash
# for linux
curl -LsSf https://astral.sh/uv/install.sh | sh
# for mac
brew install uv
```

2. Set up environment variables:

Edit `.env` with your API keys (e.g. OPENAI_API_KEY). NOTE: an OpenAI api key is required.

```bash
cp .env.example .env
```

2. Run an agent:

```bash
uv run src/run.py spot
```

NOTE: `uv` does many things in the background, such as setting up a good `venv` and downloading any dependencies if needed. Please add new dependencies to `pyproject.toml`.

NOTE: If you are running complex models, or need to download dependencies, there may be a delay before the agent starts.

## CLI Commands

The main entry point is `src/run.py` which provides the following commands:

- `start`: Start an agent with a specified config
  ```bash
  python src/run.py start [config_name] [--debug]
  ```
  - `config_name`: Name of the config file (without .json extension) in the config directory
  - `--debug`: Optional flag to enable debug logging

## Developer Guide

### Project Structure

```
.
├── config/                 # Agent configuration files
├── src/
│   ├── fuser/            # Input fusion logic
│   ├── input/            # Input plugins (e.g. VLM, audio)
│   ├── llm/              # LLM integration
│   ├── modules/          # Agent outputs/actions/capabilities
│   ├── runtime/          # Core runtime system
│   └── run.py            # CLI entry point
```

### Adding New Modules

Modules are the core capabilities of an agent. For example, for a robot, these capabilities are actions such as movement and speech. Each module consists of:

1. Interface (`interface.py`): Defines input/output types.
2. Implementation (`impl/`): Business logic, if any. Otherwise, use passthrough.
3. Connector (`connector/`): Code that connects `omOS` to specific virtual or physical environments, typically through middleware (e.g. custom APIs, `ROS2`, `Zenoh`, or `CycloneDDS`)

Example module structure:

```
modules/
└── move_{unique_hardware_id}/
    ├── interface.py      # Defines MoveInput/Output
    ├── impl/
    │   └── passthrough.py
    └── connector/
        ├── ros2.py      # Maps omOS data/commands to other ROS2
        ├── zenoh.py
        └── unitree_LL.py
```

In general, each robot will have specific capabilities, and therefore, each module will be hardware specific.

*Example*: if you are adding support for the Unitree G1 Humanoid version 13.2b, which supports a new movement subtype such as `dance_2`, you could name the updated module `move_unitree_g1_13_2b` and select that module in your `unitree_g1.json` configuration file.

### Configuration

Agents are configured via JSON files in the `config/` directory. Key configuration elements:

```json
{
  "hertz": 0.5, // Agent base tick rate, that can be overridden to respond
                // quickly to changing environments via event triggered
                // callbacks through real time middleware
  "name": "agent_name", // Unique identifier
  "system_prompt": "...", // Agent personality/behavior
  "agent_inputs": [
    // Input sources
    {
      "type": "VlmInput" // Input plugin to use
    }
  ],
  "cortex_llm": {
    // LLM configuration
    "type": "OpenAILLM" // LLM plugin to use
  },
  "modules": [
    // Available capabilities
    {
      "name": "move", // Module name
      "impl": "passthrough", // Implementation to use
      "connector": "ros2" // Connector handler
    }
  ]
}
```

### Runtime Flow

1. Input plugins collect data (vision, audio, etc.)
2. The Fuser combines inputs into a prompt
3. The LLM generates commands based on the prompt
4. The ModuleOrchestrator executes commands through modules
5. Connectors map omOS data/commands to external data buses and data distribution systems such as custom APIs, `ROS2`, `Zenoh`, or `CycloneDDS`.

### Core operating principle of the system

The system is not event or callback driven, but is based on a loop that runs at a fixed frequency of `self.config.hertz`. This loop looks for the most recent data from various sources, fuses the data into a prompt, sends that prompt to one or more LLMs, and then sends the LLM responses to virtual or physical robots.


```python
# cortex.py
    async def _run_cortex_loop(self) -> None:
        while True:
            await asyncio.sleep(1 / self.config.hertz)
            await self._tick()

    async def _tick(self) -> None:
        finished_promises, _ = await self.module_orchestrator.flush_promises()
        prompt = self.fuser.fuse(self.config.agent_inputs, finished_promises)
        if prompt is None:
            logging.warning("No prompt to fuse")
            return
        output = await self.config.cortex_llm.ask(prompt)
        if output is None:
            logging.warning("No output from LLM")
            return

        logging.debug("I'm thinking... ", output)
        await self.module_orchestrator.promise(output.commands)
```

### Development Tips

1. Use `--debug` flag for detailed logging
2. Add new input plugins in `src/input/plugins/`
3. Add new LLM integrations in `src/llm/plugins/`
4. Test modules with the `passthrough` implementation first
5. Use type hints and docstrings for better code maintainability

## Environment Variables

Required environment variables:

- `OPENAI_API_KEY`: OpenAI API key for LLM integration

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

[Add your license information here]
