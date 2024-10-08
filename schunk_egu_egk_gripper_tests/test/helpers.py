from threading import Thread, Event
from rclpy.node import Node
from typing import Any
from schunk_egu_egk_gripper_interfaces.msg import State  # type: ignore[attr-defined]
import uuid
from rclpy.executors import MultiThreadedExecutor
import time
from rclpy.action import ActionClient


class TopicGetsPublished(Node):
    def __init__(self, topic: str, type: Any):
        self.node_name = "check_topic" + str(uuid.uuid4()).replace("-", "")
        super().__init__(self.node_name)
        self.event = Event()
        self.data = None
        self.sub = self.create_subscription(type, topic, self.msg_cb, 3)
        self.thread = Thread(target=self.spin)
        self.thread.start()

    def spin(self) -> None:
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.spin()

    def msg_cb(self, data: Any) -> None:
        self.data = data
        self.event.set()


def get_current_state(variable: str):
    topic = TopicGetsPublished("/state", State)
    topic.event.wait()
    return getattr(topic.data, variable)


class ServiceReturnsResult(Node):
    def __init__(self, service: str, type: Any, msg: Any):
        self.node_name = "call_service" + str(uuid.uuid4()).replace("-", "")
        super().__init__(self.node_name)
        self.event = Event()
        srv = self.create_client(type, service)
        srv.wait_for_service(1.0)
        self.future = srv.call_async(msg)
        self.result = None
        self.thread = Thread(target=self.spin)
        self.thread.start()

    def spin(self) -> None:
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.spin_until_future_complete(self.future)
        self.result = self.future.result()
        self.event.set()


class ActionReturnsResult(Node):
    def __init__(self, action: str, type: Any, goal: Any):
        self.node_name = "call_action" + str(uuid.uuid4()).replace("-", "")
        super().__init__(self.node_name)
        self.event = Event()

        client = ActionClient(self, type, action)
        client.wait_for_server(1.0)

        self.goal_future = client.send_goal_async(goal)
        self.result = None
        self.thread = Thread(target=self.spin)
        self.thread.start()

    def spin(self) -> None:
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.spin_until_future_complete(self.goal_future)
        goal_handle = self.goal_future.result()
        result_future = goal_handle.get_result_async()
        executor.spin_until_future_complete(result_future)
        self.result = result_future.result().result
        self.event.set()


def check_each_in(elements: list, node_method: str) -> None:
    node = Node("test")
    until_ready = 2.0  # sec
    time.sleep(until_ready)
    existing = getattr(node, node_method)()
    advertised = [i[0] for i in existing]
    for element in elements:
        assert element in advertised
