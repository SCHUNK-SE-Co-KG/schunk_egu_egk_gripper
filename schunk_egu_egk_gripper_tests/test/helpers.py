from threading import Thread, Event
import rclpy
from rclpy.node import Node
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
