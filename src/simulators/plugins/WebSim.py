import logging
import asyncio
import json
import os
from typing import List, Optional
from dataclasses import dataclass, asdict
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import uvicorn
import threading
from datetime import datetime
import time

from llm.output_model import Command
from providers.io_provider import IOProvider
from simulators.base import Simulator


@dataclass
class SimulatorState:
    inputs: dict
    current_action: str = "idle"
    last_speech: str = ""
    current_emotion: str = ""
    system_latency: dict = None

    def to_dict(self):
        return asdict(self)


class WebSim(Simulator):
    def __init__(self):
        super().__init__(name="WebSim")
        self.messages: list[str] = []
        self.io_provider = IOProvider()
        self.io_provider.fuser_end_time = 0
        self.io_provider.llm_start_time = 0
        self.io_provider.llm_end_time = 0

        self.name = __class__
        self._initialized = False
        self._lock = threading.Lock()
        self._last_tick = time.time()
        self._tick_interval = 0.1  # 100ms tick rate

        # Initialize state
        self.state = SimulatorState(
            inputs={},
            current_action="idle",
            last_speech="",
            current_emotion="",
            system_latency={"fuse_time": 0, "llm_start": 0, "processing": 0, "complete": 0}
        )

        logging.info("Initializing WebSim...")

        # Initialize FastAPI app
        self.app = FastAPI()

        # Mount assets directory
        assets_path = os.path.join(os.path.dirname(__file__), "assets")
        self.app.mount("/assets", StaticFiles(directory=assets_path), name="assets")

        self.active_connections: List[WebSocket] = []

        # Setup routes
        @self.app.get("/")
        async def get_index():
            return HTMLResponse("""
            <!DOCTYPE html>
            <html>
                <head>
                    <title>OpenMind Simulator</title>
                    <script src="https://unpkg.com/react@17/umd/react.development.js"></script>
                    <script src="https://unpkg.com/react-dom@17/umd/react-dom.development.js"></script>
                    <script src="https://unpkg.com/babel-standalone@6/babel.min.js"></script>
                    <link href="https://cdn.jsdelivr.net/npm/tailwindcss@2.2.19/dist/tailwind.min.css" rel="stylesheet">
                </head>
                <body class="bg-gray-50">
                    <div id="root"></div>
                    <script type="text/babel">
                        function App() {
                            const [state, setState] = React.useState({
                                inputs: {},
                                current_action: "idle",
                                last_speech: "",
                                current_emotion: "",
                                system_latency: {
                                    fuse_time: 0,
                                    llm_start: 0,
                                    processing: 0,
                                    complete: 0
                                }
                            });
                            const [error, setError] = React.useState(null);
                            const [connected, setConnected] = React.useState(false);
                            const [isExpanded, setIsExpanded] = React.useState(false);
                            const historyRef = React.useRef(null);

                            const toggleExpand = () => {
                                setIsExpanded(!isExpanded);
                            };

                            const scrollToBottom = () => {
                                if (historyRef.current) {
                                    historyRef.current.scrollTop = historyRef.current.scrollHeight;
                                }
                            };

                            React.useEffect(() => {
                                if (historyRef.current) {
                                    scrollToBottom();
                                }
                            }, [state.inputs]);

                            React.useEffect(() => {
                                const connectWebSocket = () => {
                                    const ws = new WebSocket('ws://localhost:8000/ws');

                                    ws.onopen = () => {
                                        console.log('Connected to WebSocket');
                                        setConnected(true);
                                        setError(null);
                                    };

                                    ws.onmessage = (event) => {
                                        try {
                                            const data = JSON.parse(event.data);
                                            console.log('Received state:', data);
                                            setState(data);
                                            setError(null);
                                        } catch (e) {
                                            console.error('Error parsing message:', e);
                                            setError('Error parsing data from server');
                                        }
                                    };

                                    ws.onerror = (event) => {
                                        console.error('WebSocket error:', event);
                                        setConnected(false);
                                        setError('Error connecting to server');
                                    };

                                    ws.onclose = () => {
                                        console.log('WebSocket closed, attempting to reconnect...');
                                        setConnected(false);
                                        setTimeout(connectWebSocket, 1000);
                                    };

                                    return () => {
                                        ws.close();
                                    };
                                };

                                connectWebSocket();
                            }, []);

                            if (error) return (
                                <div className="min-h-screen flex items-center justify-center">
                                    <div className="text-red-600 text-xl">{error}</div>
                                </div>
                            );

                            if (!connected) return (
                                <div className="min-h-screen flex items-center justify-center">
                                    <div className="text-xl">Connecting to simulator...</div>
                                </div>
                            );

                            return (
                                <div className="min-h-screen p-4">
                                    <div className="container mx-auto">
                                        <div className="grid grid-cols-12 gap-4">
                                            {/* Input History */}
                                            <div className="col-span-4 bg-white rounded-lg shadow p-4">
                                                <div className="flex justify-between items-center mb-4">
                                                    <h2 className="text-xl font-bold">Input History</h2>
                                                    <button
                                                        onClick={toggleExpand}
                                                        className="px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600 transition-colors text-sm flex items-center"
                                                    >
                                                        {isExpanded ? "Collapse ↑" : "Expand ↓"}
                                                    </button>
                                                </div>
                                                <div
                                                    ref={historyRef}
                                                    className="overflow-auto transition-all duration-300 ease-in-out"
                                                    style={{ maxHeight: isExpanded ? 'calc(100vh - 200px)' : 'calc(100vh - 300px)' }}
                                                >
                                                    <div className="space-y-2">
                                                        {Object.entries(state.inputs || {})
                                                            .sort((a, b) => b[1].timestamp - a[1].timestamp)
                                                            .map(([key, value]) => (
                                                            <div key={key} className="bg-gray-50 p-3 rounded hover:bg-gray-100 transition-colors">
                                                                <div className="flex justify-between items-start">
                                                                    <div className="text-sm text-blue-600 font-medium">
                                                                        {key.toUpperCase()}
                                                                    </div>
                                                                    <div className="text-xs text-gray-500 ml-2">
                                                                        {value.timestamp.toFixed(3)}s
                                                                    </div>
                                                                </div>
                                                                <div className="mt-2 text-gray-900 text-sm whitespace-pre-wrap break-words">
                                                                    {value.input}
                                                                </div>
                                                            </div>
                                                        ))}
                                                    </div>
                                                </div>
                                            </div>

                                            {/* Main Display */}
                                            <div className="col-span-8">
                                                <div className="bg-white rounded-lg shadow p-4 mb-4">
                                                    <h2 className="text-xl font-bold mb-4">Current State</h2>
                                                    <div className="space-y-4">
                                                        <div>
                                                            <span className="font-semibold">Action:</span>
                                                            <span className="ml-2 text-blue-600">{state.current_action}</span>
                                                        </div>
                                                        <div>
                                                            <span className="font-semibold">Last Speech:</span>
                                                            <div className="mt-1 p-2 bg-gray-50 rounded">
                                                                {state.last_speech || "No speech yet"}
                                                            </div>
                                                        </div>
                                                        <div>
                                                            <span className="font-semibold">Emotion:</span>
                                                            <span className="ml-2 text-purple-600">{state.current_emotion}</span>
                                                        </div>
                                                    </div>
                                                </div>

                                                <div className="bg-white rounded-lg shadow p-4">
                                                    <h2 className="text-xl font-bold mb-4">System Latency</h2>
                                                    <div className="space-y-2">
                                                        {Object.entries(state.system_latency || {}).map(([key, value]) => (
                                                            <div key={key} className="flex justify-between items-center">
                                                                <span className="font-semibold">{key}:</span>
                                                                <span className="text-gray-600">
                                                                    {value ? value.toFixed(3) : '0.000'}s
                                                                </span>
                                                            </div>
                                                        ))}
                                                    </div>
                                                </div>
                                            </div>
                                        </div>
                                    </div>
                                </div>
                            );
                        }

                        ReactDOM.render(<App />, document.getElementById('root'));
                    </script>
                </body>
            </html>
            """)


        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.active_connections.append(websocket)
            try:
                await websocket.send_json(self.state.to_dict())
                while True:
                    await websocket.receive_text()
            except Exception as e:
                logging.error(f"WebSocket error: {e}")
            finally:
                self.active_connections.remove(websocket)

        # Start server thread
        try:
            logging.info("Starting WebSim server thread...")
            self.server_thread = threading.Thread(target=self._run_server, daemon=True)
            self.server_thread.start()
            time.sleep(1)

            if self.server_thread.is_alive():
                logging.info("WebSim server started successfully - Open http://localhost:8000 in your browser")
                self._initialized = True
            else:
                logging.error("WebSim server failed to start")
        except Exception as e:
            logging.error(f"Error starting WebSim server thread: {e}")

    def _run_server(self):
        """Run the FastAPI server"""
        logging.info("Starting WebSim server at http://localhost:8000")
        try:
            config = uvicorn.Config(
                app=self.app,
                host="0.0.0.0",
                port=8000,
                log_level="info",
                access_log=True
            )
            server = uvicorn.Server(config)
            server.run()
        except Exception as e:
            logging.error(f"Failed to start WebSim server: {e}")

    async def broadcast_state(self):
        """Broadcast current state to all connected clients"""
        if not self.active_connections:
            return

        try:
            with self._lock:
                earliest_time = self.get_earliest_time()

                # Process system latency relative to earliest time
                current_time = time.time()
                system_latency = {
                    "fuse_time": self.io_provider.fuser_end_time - self.io_provider.fuser_start_time if self.io_provider.fuser_start_time else 0,
                    "llm_start": self.io_provider.llm_start_time - self.io_provider.fuser_end_time if self.io_provider.llm_start_time and self.io_provider.fuser_end_time else 0,
                    "processing": self.io_provider.llm_end_time - self.io_provider.llm_start_time if self.io_provider.llm_end_time and self.io_provider.llm_start_time else 0,
                    "complete": self.io_provider.llm_end_time - self.io_provider.fuser_start_time if self.io_provider.llm_end_time and self.io_provider.fuser_start_time else 0
                }

                # Process inputs directly from IOProvider
                processed_inputs = {}
                for key, input_obj in self.io_provider.inputs.items():
                    if input_obj and input_obj.input and input_obj.timestamp is not None:
                        processed_inputs[key] = {
                            "timestamp": input_obj.timestamp - earliest_time,
                            "input": input_obj.input
                        }

                state_dict = {
                    "current_action": self.state.current_action,
                    "last_speech": self.state.last_speech,
                    "current_emotion": self.state.current_emotion,
                    "system_latency": system_latency,
                    "inputs": processed_inputs
                }

            # Broadcast to all clients
            disconnected = []
            for connection in self.active_connections:
                try:
                    await connection.send_json(state_dict)
                except Exception as e:
                    logging.error(f"Error broadcasting to client: {e}")
                    disconnected.append(connection)

            for connection in disconnected:
                try:
                    self.active_connections.remove(connection)
                except ValueError:
                    pass

        except Exception as e:
            logging.error(f"Error in broadcast_state: {e}")

    def get_earliest_time(self) -> float:
        """Get earliest timestamp from inputs"""
        earliest_time = float("inf")
        for input_obj in self.io_provider.inputs.values():
            if input_obj and input_obj.timestamp is not None:
                if input_obj.timestamp < earliest_time:
                    earliest_time = input_obj.timestamp
        return earliest_time if earliest_time != float("inf") else 0.0

    def tick(self) -> None:
        """Update simulator state"""
        if self._initialized:
            try:
                # Get or create event loop
                try:
                    loop = asyncio.get_event_loop()
                except RuntimeError:
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)

                # Broadcast state
                loop.run_until_complete(self.broadcast_state())
                
                # Sleep to maintain tick rate
                time.sleep(0.1)  # 100ms tick rate
                
            except Exception as e:
                logging.error(f"Error in tick: {e}")
                time.sleep(0.1)  # Sleep even on error to maintain tick rate

    def sim(self, commands: List[Command]) -> None:
        """Handle simulation updates from commands"""
        if not self._initialized:
            logging.warning("WebSim not initialized, skipping sim update")
            return

        try:
            updated = False
            with self._lock:
                for command in commands:
                    if command.name == "move":
                        new_action = command.arguments[0].value
                        if new_action != self.state.current_action:
                            self.state.current_action = new_action
                            updated = True
                    elif command.name == "speak":
                        new_speech = command.arguments[0].value
                        if new_speech != self.state.last_speech:
                            self.state.last_speech = new_speech
                            updated = True
                    elif command.name == "face":
                        new_emotion = command.arguments[0].value
                        if new_emotion != self.state.current_emotion:
                            self.state.current_emotion = new_emotion
                            updated = True

            if updated:
                self._last_tick = 0
                self.tick()

        except Exception as e:
            logging.error(f"Error in sim update: {e}")

    def cleanup(self):
        """Clean up resources"""
        logging.info("Cleaning up WebSim...")
        self._initialized = False

        for connection in self.active_connections[:]:
            try:
                connection.close()
            except Exception as e:
                logging.error(f"Error closing connection: {e}")
        self.active_connections.clear()