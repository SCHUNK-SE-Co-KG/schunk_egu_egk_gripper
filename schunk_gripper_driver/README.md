# SCHUNK Gripper Driver


## Lifecycle behavior

The driver implements a [lifecycle node](https://github.com/ros2/demos/tree/humble/lifecycle) with defined state transitions.
This gives us advanced control over configuration and resetting processes.

### Command line
With built-in features

```bash
ros2 lifecycle get /schunk/driver
ros2 lifecycle set /schunk/driver configure  # activate | deactivate | cleanup | shutdown
```

And with ROS2 services

```bash
ros2 service call /schunk/driver/get_state lifecycle_msgs/srv/GetState '{}'
ros2 service call /schunk/driver/change_state lifecycle_msgs/srv/ChangeState '{transition: {label: configure}}'  # activate | deactivate etc.
```
