from threading import Thread, Event
import rclpy
from rclpy.node import Node
from typing import Any
from schunk_egu_egk_gripper_interfaces.msg import State  # type: ignore[attr-defined]
import uuid


class TopicGetsPublished(Node):
    def __init__(self, topic: str, type: Any):
        node_name = "check_topic" + str(uuid.uuid4()).replace("-", "")
        super().__init__(node_name)
        self.event = Event()
        self.data = None
        self.sub = self.create_subscription(type, topic, self.msg_cb, 3)
        self.thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.thread.start()

    def msg_cb(self, data: Any) -> None:
        self.data = data
        self.event.set()


def get_current_state(variable: str):
    topic = TopicGetsPublished("/state", State)
    topic.event.wait()
    return getattr(topic.data, variable)
