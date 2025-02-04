# Openmind OS (OM1)

Openmind's OM1 is an agent runtime system that enables the creation and execution of digital and physical embodied AI agents with modular capabilities like movement, speech, and perception. One benefit of using OM1 is the ease of deploying consistent digital personas across virtual and physical environments.

## Overview

- [Core Architecture and Runtime Flow](#core-architecture-and-runtime-flow)
- [Quick Start](#quick-start)
- [Examples](#examples--gemini-and-voice-inputs--conversation-)
  * [Example 1 - Using DeepSeek or Gemini as the Core LLM](#example-1---using-deepseek-or-gemini-as-the-core-llm)
  * [Example 2 - Using Cloud Endpoints for Voice Inputs and Text to Speech](#example-2---using-cloud-endpoints-for-voice-inputs-and-text-to-speech)
- [Developer Guide](https://github.com/OpenmindAGI/OM1/blob/main/documention/developer_guide.md)
- [Unitree Dog](https://github.com/OpenmindAGI/OM1/blob/main/documention/unitree_robot.md)
- [Unitree Dog with Crypto Wallet](https://github.com/OpenmindAGI/OM1/blob/main/documention/coinbase_hackathon.md)
- [Contributing](#contributing)
- [License](#license)

## Core Architecture and Runtime Flow

The system is based on a loop that runs at a fixed frequency of `self.config.hertz`. This loop looks for the most recent data from various sources, fuses the data into a prompt, sends that prompt to one or more LLMs, and then sends the LLM responses to virtual agents or physical robots.

Specific runtime flow:

1. Input plugins collect data (vision, audio, etc.)
2. The Fuser combines inputs into a prompt
3. The LLM generates commands based on the prompt
4. The ActionOrchestrator executes commands through actions
5. Connectors map OM1 data/commands to external data buses and data distribution systems such as custom APIs, `ROS2`, `Zenoh`, or `CycloneDDS`.


```python
# /src/runtime/cortex.py
async def _run_cortex_loop(self) -> None:
    while True:
        await asyncio.sleep(1 / self.config.hertz)
        await self._tick()

async def _tick(self) -> None:
    finished_promises, _ = await self.action_orchestrator.flush_promises()
    prompt = self.fuser.fuse(self.config.agent_inputs, finished_promises)
    output = await self.config.cortex_llm.ask(prompt)
    logging.debug("I'm thinking... ", output)
    await self.action_orchestrator.promise(output.commands)
```

## Quick Start

1. Install the Rust python package manager `uv`:

```bash
# for linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# for mac
brew install uv
```

2. Clone the repo

```bash
git clone https://github.com/OpenmindAGI/OM1.git
cd OM1
git submodule update --init 
uv venv
```

3. Set up environment and configuration variables

Add your Openmind API key in `/config/spot.json`. You can obtain an free Openmind access key at http://openmind.org/free. If you leave the placeholder, `openmind-free`, you may be rate limited. 

```bash
# /config/spot.json`
...
"api_key": "openmind-free"
...
```

> [!NOTE]
> You can directly access other OpenAI style endpoints by specifying a custom API endpoint in your configuration file. To do this provide an suitable `base_url` and the `api_key` for OpenAI, DeepSeek, or other providers. Possible `base_url` choices are:
>   - https://api.openai.com/v1
>   - https://api.deepseek.com/v1
>   - https://generativelanguage.googleapis.com/v1beta/openai/

4. Run Spot, a `Hello World` agent

This basic agent uses webcam data to estimate your emotion, generates a dummy VLM caption (*DUMMY VLM - FAKE DATA - I see {random_int_num} people. Also, I see a rocket.*), looks up an Ethereum wallet balance, and sends those inputs to a central LLM. The LLM then returns `movement`, `speech`, and `face` commands, which are displayed in `RacoonSim`, a small `pygame` window. `RacooonSim` also shows basic timing and other debug information. **NOTE**: The simulator shows you generated speech, but does not send anything to your computer's audio hardware, reducing the need for you to install audio drivers. 

```bash
uv run src/run.py spot
```

Add ` --debug` to see more logging information. 

> [!NOTE]
> * `uv` does many things in the background, such as setting up a `venv` and downloading any dependencies if needed. Please add new dependencies to `pyproject.toml`.
> * If you are running complex models, or need to download dependencies, there may be a delay before the agent starts.
> * There should be a `pygame` window that pops up when you run `uv run src/run.py spot`. If you do not see `RacoonSim`, the window might be hidden behind all your other open windows - use "show all windows" (or equivalent) to find it.

## Examples: Gemini and Voice Inputs (conversation)

### Example 1 - Using DeepSeek or Gemini as the Core LLM

These preconfigured examples are similar to the `Hello World (Spot)` example, except they use `DeepSeek` or `Gemini`rather than `OpenAI 4o`.

```bash
uv run src/run.py deepseek
uv run src/run.py gemini
```

### Example 2 - Using Cloud Endpoints for Voice Inputs and Text to Speech

This preconfigured example uses your `default` audio in (microphone) and your `default` audio output (speaker). Please test both your microphone and speaker in your system settings to make sure they are connected and working.

```bash
uv run src/run.py conversation
```

You will be able to speak to the LLM, and it will generate voice outputs.

> [!NOTE]
> * On Mac and Linux, you may need to install `portaudio`. Mac: `brew install portaudio`, Linux: `sudo apt-get update && sudo apt-get install portaudio19-dev python-all-dev`.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

[Add your license information]
