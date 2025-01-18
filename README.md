# OpenMind OS (omOS)

OpenMind OS is an agent runtime system that enables the creation and execution of embodied AI agents with modular capabilities like movement, speech, and perception.

## Quick Start

1. Set up environment variables:

```bash
cp .env.example .env
# Edit .env with your API keys (e.g. OPENAI_API_KEY)
```

and download modules. To install `omOS-modules`, please clone the omOS-modules repo. It can live anywhere in your filesystem. Activate the `uv` environment and install the modules via 

```bash
uv pip install -e dir
```

At that point, `uv` invoked elsewhere, such as in `omOS`, will be able to find those modules. 

3. Run an agent:

```bash
uv run src/run.py spot
```

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
│   ├── modules/          # Agent capabilities
│   ├── runtime/          # Core runtime system
│   └── run.py            # CLI entry point
```

### Adding New Modules

Modules are the core capabilities of an agent. Each module consists of:

1. Interface (`interface.py`): Defines input/output types
2. Implementation (`impl/`): Business logic
3. Mutation (`mutation/`): Side effects (e.g. ROS2 commands)

Example module structure:

```
modules/
└── move/
    ├── interface.py      # Defines MoveInput/Output
    ├── impl/
    │   └── passthrough.py
    └── mutation/
        └── ros2.py
```

### Configuration

Agents are configured via JSON files in the `config/` directory. Key configuration elements:

```json
{
  "hertz": 0.5, // Agent tick rate
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
      "mutation": "ros2" // Mutation handler
    }
  ]
}
```

### Runtime Flow

1. Input plugins collect data (vision, audio, etc.)
2. The Fuser combines inputs into a prompt
3. The LLM generates commands based on the prompt
4. The ModuleOrchestrator executes commands through modules
5. Mutations handle side effects (e.g. ROS2 commands)

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
