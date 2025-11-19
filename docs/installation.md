## Installation

1. Clone the repository into your ROS2 workspace and install system dependencies:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/SCHUNK-SE-Co-KG/schunk_mechatronic_gripper.git schunk_gripper
    cd ..
    sudo apt update -qq
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

2. (Optional) Create and activate a Python virtual environment for the library dependencies:
    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    ```

3. Install the Python library dependencies:
    ```bash
    cd src/schunk_gripper/schunk_gripper_library
    pip install -e .
    ```

4. Build the workspace:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

5. If `colcon build` fails due to missing Python packages, install them manually:
    ```bash
    pip install empy catkin_pkg lark
    ```
