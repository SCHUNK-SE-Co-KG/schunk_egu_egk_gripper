import pytest
from test.conftest import launch_description
from test.helpers import check_each_in
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from schunk_egu_egk_gripper_interfaces.action import (  # type: ignore[attr-defined]
    MoveToAbsolutePosition,
)


@pytest.mark.launch(fixture=launch_description)
def test_driver_advertices_all_relevant_actions(
    launch_context, isolated, gripper_dummy
):
    action_list = [
        "/grip",
        "/grip_with_position",
        "/gripper_control",
        "/move_to_absolute_position",
        "/move_to_relative_position",
        "/release_workpiece",
    ]
    action_list = [a + "/_action/status" for a in action_list]
    check_each_in(action_list, "get_topic_names_and_types")


@pytest.mark.launch(fixture=launch_description)
def test_driver_moves_to_absolute_position(launch_context, isolated, gripper_dummy):
    node = Node("move_test")
    time.sleep(2)

    client = ActionClient(node, MoveToAbsolutePosition, "/move_to_absolute_position")
    client.wait_for_server()
    goal = MoveToAbsolutePosition.Goal()

    goal_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, goal_future)
    goal_handle = goal_future.result()

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result
    assert result.position_reached
