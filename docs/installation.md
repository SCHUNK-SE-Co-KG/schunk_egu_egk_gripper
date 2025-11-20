# Installation

You can install the SCHUNK Mechatronic Gripper either in a ready-to-use Docker container (recommended) or directly on your host system.

## Install with Docker (Recommended)

To run the SCHUNK Mechatronic Gripper in a container (pre-built and ready to use):

1. Clone this project and build the Docker image (from the project root):

```bash
git clone https://github.com/SCHUNK-SE-Co-KG/schunk_mechatronic_gripper.git
docker build -t schunk_gripper .
```

2. Run the container:

```bash
docker run -it \
    --net=host \
    --env=ROS_LOCALHOST_ONLY=1 \
    -v /dev:/dev \
    --privileged \
    schunk_gripper \
    ros2 launch schunk_gripper_driver driver.launch.py
```
> **Note:** See [launching](launching.md) for available launch commands.  
> Adjust the launch arguments to fit your setup.  
> For example, if you do not use Modbus grippers via serial, you can omit the `-v /dev:/dev` and `--privileged` options.


## Install on Host System


Follow these steps if you want to run the SCHUNK Mechatronic Gripper directly on your host machine.

1. Clone this project and install system dependencies:
```bash
cd ~/schunk_gripper_ws/src
git clone https://github.com/SCHUNK-SE-Co-KG/schunk_mechatronic_gripper.git .
cd ..
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y
```

2. (Optional) Create and activate a Python virtual environment:
```bash
python3 -m venv .venv
source .venv/bin/activate
```

3. Install the Python library dependencies:
```bash
pip install -e src/schunk_gripper_library
```

4. Build the ROS2 workspace:
```bash
cd ~/schunk_gripper_ws
colcon build
```

5. Launch the driver:

```bash
ros2 launch schunk_gripper_driver driver.launch.py
```
> **Note:** See [launching](launching.md) for available launch commands.  