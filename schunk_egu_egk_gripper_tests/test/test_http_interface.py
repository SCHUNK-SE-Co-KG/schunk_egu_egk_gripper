import pytest
from test.conftest import launch_description
import time
from threading import Thread, Event
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from typing import Any


class CheckTopic(Node):
    def __init__(self, topic: str, type: Any):
        super().__init__("check_topic")
        self.event = Event()
        self.sub = self.create_subscription(type, topic, self.msg_cb, 3)
        self.thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.thread.start()

    def msg_cb(self, data: Any) -> None:
        self.event.set()


@pytest.mark.launch(fixture=launch_description)
def test_driver_connnects_to_gripper_dummy(launch_context, isolated, gripper_dummy):
    until_dummy_ready = 3
    timeout = 3

    time.sleep(until_dummy_ready)
    assert CheckTopic("/EGK_50_M_B/joint_states", JointState).event.wait(timeout)
