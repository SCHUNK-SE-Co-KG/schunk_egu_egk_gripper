
<div align="center">
  <img src="resources/images/schunk_egu.png" alt="Schunk Gripper" style="width: 25%;"/>
  <h1 align="center">SCHUNK Gripper</h1>
</div>

<p align="center">
  <a href="https://opensource.org/licenses/gpl-license">
    <img src="https://img.shields.io/badge/License-GPLv3-orange.svg" alt="License">
  </a>
  <a href="https://github.com/SCHUNK-SE-Co-KG/schunk_egu_egk_gripper/actions">
    <img src="https://github.com/SCHUNK-SE-Co-KG/schunk_egu_egk_gripper/actions/workflows/industrial_ci_humble_action.yml/badge.svg" alt="build badge humble">
  </a>
  <a href="https://github.com/SCHUNK-SE-Co-KG/schunk_egu_egk_gripper/actions">
    <img src="https://github.com/SCHUNK-SE-Co-KG/schunk_egu_egk_gripper/actions/workflows/industrial_ci_jazzy_action.yml/badge.svg" alt="build badge jazzy">
  </a>
</p>


This is the ROS2 driver for SCHUNK's
[EGU](https://schunk.com/us/en/gripping-systems/parallel-gripper/egu/c/PGR_6556),
[EGK](https://schunk.com/us/en/gripping-systems/parallel-gripper/egk/c/PGR_6557),
and [EZU](https://schunk.com/us/en/gripping-systems/centric-grippers/ezu/c/PGR_7387) mechatronic grippers.
The driver supports both _Modbus RTU_ and _Ethernet_-based (_PROFINET_, _Ethernet/IP_, and _EtherCAT_) versions.


## Under construction
We are currently doing a complete rework of this driver in Python.
A feature-complete release is planned for end of October '25.

## Build and install
1. In a new terminal, source your global ROS2 environment, e.g.
    ```bash
    source /opt/ros/humble/setup.bash
    ```
2. Navigate into your local ROS2 workspace, and install ROS2 dependencies with
    ```bash
    git clone https://github.com/SCHUNK-SE-Co-KG/schunk_egu_egk_gripper.git src/schunk_gripper
    sudo apt update -qq
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

2. You'll need some additional python dependencies that are best managed
    inside a _virtual environment_.
    After creating and sourcing such an environment, navigate into the `schunk_gripper_library` subfolder and
    ```bash
    pip install .
    ```
    This will install everything that the driver requires.

3. (Optional): Only if the `colcon build` command from below fails:
    ```bash
    pip install empy catkin_pkg lark
    ```

4. Finally, inside your ROS2 workspace, build the driver with
    ```bash
    colcon build
    ```


## Getting started
In every new terminal, source your local ROS2 workspace
```bash
source install/setup.bash
```
and your _virtual environment_ with
```bash
source .venv/bin/activate
```

You can then start the driver with
```bash
ros2 launch schunk_gripper_driver driver.launch.py
```
