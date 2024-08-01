import pytest
import rclpy
from rclpy.node import Node
from test.conftest import launch_description
from rclpy.action import ActionClient
from schunk_egu_egk_gripper_interfaces.action import (  # type: ignore[attr-defined]
    MoveToAbsolutePosition,
)
from schunk_egu_egk_gripper_interfaces.srv import (  # type: ignore[attr-defined]
    Acknowledge,
)
from test.helpers import get_current_state


@pytest.mark.launch(fixture=launch_description)
def test_driver_starts_in_ready_state(launch_context, isolated, gripper_dummy):
    assert get_current_state(variable="ready_for_operation") is True


@pytest.mark.launch(fixture=launch_description)
def test_driver_is_ready_after_acknowledge(launch_context, isolated, gripper_dummy):
    node = Node("test")
    activate_srv = node.create_client(Acknowledge, "/acknowledge")
    while not activate_srv.wait_for_service(1.0):
        pass
    future = activate_srv.call_async(Acknowledge.Request())
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success is True


@pytest.mark.launch(fixture=launch_description)
@pytest.mark.skip()
def test_driver_moves_to_absolute_position(launch_context, isolated, gripper_dummy):
    node = Node("move_test")
    # activate_srv = node.create_client(Acknowledge, "/acknowledge")
    # future = activate_srv.call_async(Acknowledge.Request())
    # rclpy.spin_until_future_complete(node, future)

    client = ActionClient(node, MoveToAbsolutePosition, "/move_to_absolute_position")
    client.wait_for_server()
    goal = MoveToAbsolutePosition.Goal()
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    print("done :)")
    assert False
