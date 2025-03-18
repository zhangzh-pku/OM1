# Gazebo_simulation
Repo for quadruped simulation and control with OM1.

# Simulation Instructions
## MacOS (Gazebo Harmonic)
```
cd gazebo_sim
bash macOS.sh
```
The script also checks and installs Gazebo Harmonic https://gazebosim.org/docs/harmonic/getstarted/ if not already installed.

## Ubuntu (Gazebo Classic)
```
cd gazebo_sim
bash ubuntu.sh
```
For now, ROS2 is required for the simulation to interact with OM1 in Gazebo Classic, check https://docs.ros.org/en/humble/Installation.html for installation instructions.
ffmpeg is required for streaming the simulated video feed to OM1. If your system doesn't have `ffmpeg`, you should install it using `sudo apt-get install ffmpeg` to run the program.
_Note1:_ The simulated camera feed is streamed to a virtual video device at `/dev/video10`, thus requiring the `camera_index` in the VLM config to be 10. Modify as needed if the virtual camera needs to be reordered or if OM1 needs to read from another camera.
_Note2:_ It is also possible to run the Gazebo Harmonic simulation on Ubuntu.

# Start OM1
Boot OM1 using the example config, remember to set the correct API key
```
uv run src/run.py quadruped_sim
```