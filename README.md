# Openmind OS (OM1)

Openmind's OM1 is an agent runtime system that enables the creation and execution of digital and physical embodied AI agents with modular capabilities like movement, speech, and perception. One benefit of using OM1 is the ease of deploying consistent digital personas across virtual and physical environments.

## Quick Start

1. Clone the repo

```bash
git clone https://github.com/OpenmindAGI/omOS.git
git submodule update --init --recursive
```

2. Install the Rust python package manager `uv`:

```bash
# for linux
curl -LsSf https://astral.sh/uv/install.sh | sh
# for mac
brew install uv
```

If you are on Mac, you may need to install `pyaudio` manually, via `brew install portaudio`.

3. Set up environment and configuration variables

Add your Openmind API key in `/config/spot.json`. You can obtain an free Openmind access key at http://openmind.org/free.

```bash
# /config/spot.json`
...
"api_key": "openmind-pat-cnh4d7_example_797wtgy47d"
...
```

> [!NOTE]
> You can directly access other OpenAI style endpoints by specifying a custom API endpoint in your configuration file. To do this:
> * provide an alternative `base_url`, such as: 
> - https://api.openai.com/v1
> - https://api.deepseek.com/v1
> - https://generativelanguage.googleapis.com/v1beta/openai/
> * change the configiration file `api_key:` to the OpenAI, DeepSeek, or other keys

4. Run an Hello World agent

This basic agent uses webcam data to estimate your emotion, generates a fake VLM caption, and sends those two inputs to a central LLM. The LLM then returns `movement`, `speech`, and `face` commands, which are displayed in `RacoonSim`, a small `pygame` window. `RacooonSim` also shows basic timing and other debug information.

```bash
uv run src/run.py spot
```

> [!NOTE]
> * `uv` does many things in the background, such as setting up a good `venv` and downloading any dependencies if needed. Please add new dependencies to `pyproject.toml`.
> * If you are running complex models, or need to download dependencies, there may be a delay before the agent starts.
> * There should be a `pygame` window that pops up when you run `uv run src/run.py spot`. If you do not see `RacoonSim`, the window might be hidden behind all your other open windows - use "show all windows" (or equivalent) to find it.

## Examples: Wallets, DeepSeek, and Voice Inputs (conversation)

### Example 1 - The Coinbase Wallet

Similar to the `Hello World (Spot)` example, except uses the Coinbase wallet rather than Ethereum Mainnet.

```bash
cp .env.example .env
# then, enter your coinbase credentials into the .env
# then, run
uv run src/run.py coinbase
```

The agent tracks the balance of testnet ETH in a Coinbase wallet and sends a message when there is a new transaction. The agent can be instructed via the prompt to express appreciation for receiving tokens. See `/config/coinbase.json` for an example for how this is done:

```bash
"system_prompt": "
... 
You like receiving ETH. If you receive an ETH transaction, show your appreciation though actions and speech. 
...
4. If there is a new ETH transaction, you might:\n    Move: 'shake paw'\n    Speak: {{'sentence': 'Thank you I really appreciate the ETH you just sent.'}}\n    Face: 'smile'\n\n
...",
```

The Coinbase wallet currently supports Base Sepolia and Base Mainnet networks. The Coinbase Wallet integration requires the following environment variables:

- `COINBASE_WALLET_ID`: The ID for the Coinbase Wallet.
- `COINBASE_API_KEY`: The API key for the Coinbase Project API.
- `COINBASE_API_SECRET`: The API secret for the Coinbase Project API.

The API_KEY and API_SECRET are generated from the [Coinbase Developer Portal](https://portal.cdp.coinbase.com) by navigating to the "API Keys" tab and then clicking "Create API Key". If you don't already have a Developer-Managed Wallet, you can create one by following [these instructions](https://docs.cdp.coinbase.com/mpc-wallet/docs/quickstart#creating-a-wallet) with the API key and secret you just created. Then, you can get a Wallet ID from the created wallet.

These keys are all strings and should look like this:
```bash
COINBASE_WALLET_ID="xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
COINBASE_API_KEY="organizations/your-org-id/apiKeys/your-api-key-id"
COINBASE_API_SECRET="-----BEGIN EC PRIVATE KEY-----\nyour-api-key-private-key\n-----END EC PRIVATE KEY-----\n"
```

For more details, please see the [Coinbase documentation](https://docs.cdp.coinbase.com/mpc-wallet/docs/wallets).

### Example 2 - Using DeepSeek or Gemini as the Core LLM

Similar to the `Hello World (Spot)` example, except uses `DeepSeek` or `Gemini`rather than `OpenAI 4o`.

```bash
uv run src/run.py deepseek
uv run src/run.py gemini
```

### Example 3 - Using Cloud Endpoints for Voice Inputs

```bash
uv run src/run.py conversation
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
├── config/               # Agent configuration files
├── src/
│   ├── actions/          # Agent outputs/actions/capabilities
│   ├── fuser/            # Input fusion logic
│   ├── inputs/           # Input plugins (e.g. VLM, audio)
│   ├── llm/              # LLM integration
│   ├── providers/        # ????
│   ├── runtime/          # Core runtime system
│   ├── simulators/       # Virtual agents/robots such as `RacoonSim`
│   └── run.py            # CLI entry point
```

### Adding New Actions

Actions are the core capabilities of an agent. For example, for a robot, these capabilities are actions such as movement and speech. Each action consists of:

1. Interface (`interface.py`): Defines input/output types.
2. Implementation (`implementation/`): Business logic, if any. Otherwise, use passthrough.
3. Connector (`connector/`): Code that connects `OM1` to specific virtual or physical environments, typically through middleware (e.g. custom APIs, `ROS2`, `Zenoh`, or `CycloneDDS`)

Example action structure:

```
actions/
└── move_{unique_hardware_id}/
    ├── interface.py      # Defines MoveInput/Output
    ├── implementation/
    │   └── passthrough.py
    └── connector/
        ├── ros2.py      # Maps OM1 data/commands to other ROS2
        ├── zenoh.py
        └── unitree_LL.py
```

In general, each robot will have specific capabilities, and therefore, each action will be hardware specific.

*Example*: if you are adding support for the Unitree G1 Humanoid version 13.2b, which supports a new movement subtype such as `dance_2`, you could name the updated action `move_unitree_g1_13_2b` and select that action in your `unitree_g1.json` configuration file.

### Configuration

Agents are configured via JSON files in the `config/` directory. Key configuration elements:

```json
{
  "hertz": 0.5,
  "name": "agent_name",
  "system_prompt": "...",
  "agent_inputs": [
    {
      "type": "VlmInput"
    }
  ],
  "cortex_llm": {
    "type": "OpenAILLM",  
    "config": {
      "base_url": "",
      "api_key": "your_key_here"
    }
  },
  "simulators": [
    {
      "type": "RacoonSim"
    }
  ],
  "agent_actions": [
    {
      "name": "move",
      "implementation": "passthrough",
      "connector": "ros2"
    }
  ]
}
```

* **Hertz** Defines the base tick rate of the agent. This rate can be overridden to allow the agent to respond quickly to changing environments using event-triggered callbacks through real-time middleware.

* **Name** A unique identifier for the agent.

* **System Prompt** Defines the agent’s personality and behavior. This acts as the system prompt for the agent’s operations.

* **Cortex LLM** Configuration for the language model (LLM) used by the agent.

  - **Type**: Specifies the LLM plugin.

  - **Config**: Configuration for the LLM, including the API endpoint and API key. If you do not change the file, and use the `openmind_free`, the LLM operates with a rate limiter with the OpenMind's public endpoint.

OpenMind OpenAI Proxy endpoint is [https://api.openmind.org/api/core/openai](https://api.openmind.org/api/core/openai)
OpenMind DeepSeek Proxy endpoint is [https://api.openmind.org/api/core/deepseek](https://api.openmind.org/api/core/deepseek)
OpenMind Gemini Proxy endpoint is [https://api.openmind.org/api/core/gemini](https://api.openmind.org/api/core/gemini)

```json
"cortex_llm": {
  "type": "OpenAILLM",
  "config": {
    "base_url": "...", // Required: URL of the LLM endpoint
    "api_key": "..."   // Required: API key from OpenMind or OpenAI
  }
}
```

#### Simulators

Lists the simulation modules used by the agent. These define the simulated environment or entities the agent interacts with.

```json
"simulators": [
  {
    "type": "RacoonSim"
  }
]
```

#### Agent Actions

Defines the agent’s available capabilities, including action names, their implementation, and the connector used to execute them.

```json
"agent_actions": [
  {
    "name": "move", // Action name
    "implementation": "passthrough", // Implementation to use
    "connector": "ros2" // Connector handler
  }
]
```

### Runtime Flow

1. Input plugins collect data (vision, audio, etc.)
2. The Fuser combines inputs into a prompt
3. The LLM generates commands based on the prompt
4. The ActionOrchestrator executes commands through actions
5. Connectors map OM1 data/commands to external data buses and data distribution systems such as custom APIs, `ROS2`, `Zenoh`, or `CycloneDDS`.

### Development Tips

1. Use `--debug` flag for detailed logging
2. Add new input plugins in `src/input/plugins/`
3. Add new LLM integrations in `src/llm/plugins/`
4. Test actions with the `passthrough` implementation first
5. Use type hints and docstrings for better code maintainability
6. Run `uv run ruff check . --fix` and `uv run black .` check/format your code. 

## Optional Environment Variables

- `ETH_ADDRESS`: The Ethereum address of agent, prefixed with `Ox`. Example: `0xd8dA6BF26964aF9D7eEd9e03E53415D37aA96045`. Only relevant if your agent has a wallet.
- `UNITREE_WIRED_ETHERNET`: Your netrowrk adapet that is conncted to a Unitree robot. Example: `eno0`. Only relevant if your agent has a physical (robot) embodiment.

### Core operating principle of the system

The system is based on a loop that runs at a fixed frequency of `self.config.hertz`. This loop looks for the most recent data from various sources, fuses the data into a prompt, sends that prompt to one or more LLMs, and then sends the LLM responses to virtual agents or physical robots.

```python
# cortex.py
    async def _run_cortex_loop(self) -> None:
        while True:
            await asyncio.sleep(1 / self.config.hertz)
            await self._tick()

    async def _tick(self) -> None:
        finished_promises, _ = await self.action_orchestrator.flush_promises()
        prompt = self.fuser.fuse(self.config.agent_inputs, finished_promises)
        if prompt is None:
            logging.warning("No prompt to fuse")
            return
        output = await self.config.cortex_llm.ask(prompt)
        if output is None:
            logging.warning("No output from LLM")
            return

        logging.debug("I'm thinking... ", output)
        await self.action_orchestrator.promise(output.commands)
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

[Add your license information]
