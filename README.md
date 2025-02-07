# Openmind OS (OM1)

Openmind's OM1 is an AI runtime for agents and robots with modular capabilities like movement, speech, and perception.

OM1 allows AI agents to be configured and then deployed in both digital and physical world. You can create *one* AI agent and run it in the cloud but also on physical robot hardware such as quadrupeds, and, soon, TurtleBot 3 and Humanoids. 

This means that one AI agent with a defined persona can ingest data from multiple sources (the web, X/Twitter, cameras, LIDAR, GPS, the NYSE, ...), and can then Tweet and explore your house, shake you hand, or talk to you. With OM1, you can talk with OpenAI's `gpt-4o` and literally shake hands with it.

Capabilities of OM1:

* Simple, Modular architecture
* All python 
* Easy to add new data inputs
* Easy to support new hardware, such as API endpoints and robots
* Can be connected to ROS2, Zenoh, and CycloneDDS
* Includes a simple web-based debug display to watch the system work 
* Preconfigured endpoints for Voice-to-Speech, OpenAI's `gpt-4o`, DeepSeek, and multiple VLMs

## Hello World

The `Spot` agent uses your webcam to label objects and sends those captions to `OpenAI 4o`. The LLM then returns `movement`, `speech`, and `face` commands, which are displayed in `RacoonSim`. `RacooonSim` also shows basic timing and other debug information.

1. Clone the repo

```bash clone repo
git clone https://github.com/OpenmindAGI/OM1.git
cd OM1
git submodule update --init
uv venv
```

Note: If you don't have the Rust python package manager `uv`, please install it via `brew install uv` (for Mac) and `curl -LsSf https://astral.sh/uv/install.sh | sh` for Linux.

2. Set configuration variables

Add your Openmind API key in `/config/spot.json`. You can obtain a free access key at https://portal.openmind.org/. If you use the placeholder, `openmind-free`, you may be rate limited.

```bash set api key
# /config/spot.json`
...
"api_key": "openmind_om1_pat_2f1cf005af........."
...
```

3. Run Spot, a `Hello World` agent

```bash run spot
uv run src/run.py spot
```

Add `--debug` to see more logging information.

## Detailed Documentation

More detailed documentation can be accessed at [docs.openmind.org](https://docs.openmind.org/introduction) and in this repo:

Highlights:

- [Core Architecture and Runtime Flow](./docs/development//architecture.mdx)
- [Quick Start](./docs/quick_start.mdx)
- [Developer Guide](./docs/development/guide.mdx)
- [Example 1 - Using DeepSeek or Gemini as the Core LLM](./docs/examples/llm_models.mdx)
- [Example 2 - Using Cloud Endpoints for Voice Inputs and Text to Speech](./docs/examples/conversation.mdx)
- [Example 3 - An interactive Smart Toy](./docs/examples/smart_toy.mdx)
- [Robotics - Unitree Dog](./docs/robotics/unitree_robotics.mdx)
- [Robotics - Unitree Dog with Crypto Wallet](./docs/robotics/coinbase_hackathon.mdx)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the terms of the MIT license.
