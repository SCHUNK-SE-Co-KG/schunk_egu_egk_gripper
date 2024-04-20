
<div align="center">
  <img src="resources/images/schunk_egu.png" alt="Schunk EGU/EGK Gripper" style="width: 25%;"/>
  <h1 align="center">Schunk EGU/EGK Gripper</h1>
</div>

<p align="center">
  <a href="https://opensource.org/licenses/gpl-license">
    <img src="https://img.shields.io/badge/License-GPLv3-orange.svg" alt="License">
  </a>
  <a href="https://github.com/SCHUNK-SE-Co-KG/schunk_egu_egk_gripper/actions">
    <img src="https://github.com/SCHUNK-SE-Co-KG/schunk_egu_egk_gripper/actions/workflows/industrial_ci_humble_action.yml/badge.svg" alt="build badge humble">
  </a>
  <a href="https://github.com/SCHUNK-SE-Co-KG/schunk_egu_egk_gripper/actions">
    <img src="https://github.com/SCHUNK-SE-Co-KG/schunk_egu_egk_gripper/actions/workflows/industrial_ci_iron_action.yml/badge.svg" alt="build badge iron">
  </a>
  <a href="https://github.com/SCHUNK-SE-Co-KG/schunk_egu_egk_gripper/actions">
    <img src="https://github.com/SCHUNK-SE-Co-KG/schunk_egu_egk_gripper/actions/workflows/industrial_ci_rolling_action.yml/badge.svg" alt="build badge rolling">
  </a>
</p>


This is the ROS2 driver for the SCHUNK
[EGU](https://schunk.com/us/en/gripping-systems/parallel-gripper/egu/c/PGR_6556)
and
[EGK](https://schunk.com/us/en/gripping-systems/parallel-gripper/egk/c/PGR_6557)
grippers. The driver communicates via the _AnybusCom 40_ interface and is
compatible with
grippers featuring _PROFINET_, _Ethernet/IP_, and _EtherCAT_.
The driver currently supports most gripper functionalities, except for the jog mode.

## System dependencies

We use the _curl_ library for the _HTTP_-based communication to the devices. Install that system-wide with
```bash
sudo apt install curl libcurl4-openssl-dev
```

## Build and install
Switch to the `src` folder of your current ROS2 workspace and

```bash
git clone https://github.com/SCHUNK-SE-Co-KG/schunk_egu_egk_gripper.git
sudo apt update -qq
rosdep update
rosdep install --from-paths ./ --ignore-src -y
cd ..
colcon build --symlink-install --packages-select schunk_egu_egk_gripper_driver schunk_egu_egk_gripper_library schunk_egu_egk_gripper_interfaces  
```

## Getting started
First, you'll need the IP address of your EGU/EGK gripper. The grippers should ship with _DHCP_ by default.
Let's say your own IP address is `192.168.0.3`.
You can then scan your local network to find the device with
```bash
nmap -sP -A 192.168.0.1/24
```

Now adjust the `schunk.launch.py` file in the driver's repository with the gripper's _IP_, such as
```
 parameters=[ {'IP': '192.168.0.4'} ]
```
Replace `192.168.0.4` with the IP address you found for your gripper.

You should now be able to start the launch file and interact with the gripper
```bash
source install/setup.bash
ros2 launch schunk_egu_egk_gripper_driver schunk.launch.py
```

In the launch file, you can also adjust the frequencies of the 'joint_states', 'state', and 'diagnostics' topics.

| topic             | parameter frequency                                  |
| ------            | ------                                               |
| state             | state_frq (1.0 Hz - ca. 60 Hz)                       |
| joint_states      | rate                                                 |
| diagnostics       | diagnostics_period (**Note:** Period, not frequency) |

**Note:** The 'state' topic will always publish the fastest rate. All other topics publish either at a slower rate or at the same rate, even if a faster rate is specified in the launch file. Actions always publish at the same rate as the 'state'.

**Note:** There is also a namespace. It is recommended to set the model name as the namespace when using a single gripper. If you use multiple grippers of the same model, you can also utilize different namespaces.

### Alternative Start
The provided driver functions as a component. This implies that you can initiate the Node in the following ways:

a) Using a component manager – this process is also outlined in the launch file.

b) Within your custom executable, coupled with a (multithreaded-)executor.

If you prefer starting the node in your main function, ensure that you include "schunk_gripper_driver" in the cmake target_link_libraries. Additionally, always specify the IP address, setting the parameter overrides for "IP" to your designated IP. Alternatively, create the "SchunkGripperNode"-Component and utilize the "reconnect" service, specifying your IP.

## Actions
All functionalities of the gripper, including movement, are treated as actions. This implies that when gripping, moving, or releasing a workpiece, you need to send a goal and can receive a result or feedback. Releasing a workpiece is the only action where you send an empty goal:

- `move_to_absolute_position`
- `move_to_relative_position`
- `grip`
- `grip_with_position`
- `release_workpiece`
- `gripper_control`

In contrast to the ROS1 driver, the gripping actions in ROS 2 always have the same name (`grip` and `grip_with_pos`). The ROS 2 driver still automatically recognizes the model and sets the appropriate goal based on the model.

'gripper_control' is the only action utilizing „control_msgs/action/GripperCommand“. It can be used for absolute movement (at half the maximal velocity) and gripping with or without a position. Please note that this action only supports external gripping.

## Services

Services are functionalities that do not involve movement or occur so rapidly that feedback is unnecessary. If you require more information about the gripper during such operations, it is recommended to utilize the state topic.

- `acknowledge`
- `stop`
- `fast_stop`
- `softreset`
- `parameter_get`
- `parameter_set`
- `reconnect`
- `release_for_manual_movement`
- `prepare_for_shutdown`
- `gripper_info`

**Important:** During a soft reset, no topics will be published. This will last for approximately 7 seconds. Afterward, all publications resume, and you can modify parameters.

**Important:** If you use `prepare_for_shutdown`, you also need to shut down the gripper, which can be also done using a soft reset.

**Important:** To exit from `release_for_manual_movement`, you need to activate `fast_stop` and `acknowledge`.

All other services can be used whenever you like. (**Note:** Fast stop is an abort of movement, so it always provokes an error).

`gripper_info` publishes some information about the gripper on the terminal screen.

`reconnect` is the only method for altering the IP address during runtime. If nothing is connected to the IP address or a gripper is connected, it undergoes a change. If something else is linked to this IP, errors will occur, and the old address will be retained in such cases. Exercise caution when using this service!

With `parameter_get` and `parameter_set` you can read and set all allowed Parameter of the gripper. For getting and setting you need always the parameter instance. After that 

## Parameters

You can change parameter of the Gripper using dynamic reconfigure. Following parameters are available for change:

- `use_brk`
- `grp_pos_margin`
- `grp_prepos_delta`
- `zero_pos_ofs`
- `grp_prehold_time`
- `wp_release_delta`
- `wp_lost_dst`

For this type of parameter, you have to include the namespace: "GripperParameter."
Example: `GripperParameter.use_brk`

You have the option to change the default values in the `schunk.launch.py` file, which will be loaded when you start the node.

Optionally, you can perform some basic commands to the gripper via Parameter. In this case, we recommend trying out rqt and opening the parameter reconfigure Monitor. (Note: It may not function optimally in ROS 2; if that's the case, we recommend using the terminal instead)

## Example

To explore the capabilities of the gripper-driver, we recommend using rqt. There, you can view all topics, dynamic reconfigure parameters, and services. You can also publish messages on topics (such as action goals) or call services. To launch rqt with the node, use the schunk_rqt.launch.py:
```
ros2 launch schunk_gripper schunk_rqt.launch.py
```
Alternatively, you can launch schunk.launch and open the rqt tool separately:
```
ros2 launch schunk_gripper schunk.launch.py
rqt
```
It is known that rqt can not get the message class for action feedback and action goals.
![rqt](resources/images/rqt_interface.png)

Open:
- Plugins/Configuration/Dynamic Reconfigure: For changing parameters.
- Plugins/Services/Service Caller: For calling services.
- Plugins/Topic/Topic Monitor: For viewing all messages.

Additionally, you can refer to 'gripper_example.cpp' for guidance on using this driver in your code. To run the example, start 'schunk.launch.py' (or 'schunk_rqt_launch.py') and then execute the example:  
**!!!WARNING!!! This will move the gripper jaws**
```
ros2 run schunk_gripper schunk_example
```
