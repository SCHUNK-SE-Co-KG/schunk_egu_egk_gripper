#!/usr/bin/env python3
import pytest
from rclpy.node import Node
import time
from test.conftest import launch_description


def check_each_in(elements: list, node_method: str) -> None:
    node = Node("test")
    until_ready = 2.0  # sec
    time.sleep(until_ready)
    existing = getattr(node, node_method)()
    advertised = [i[0] for i in existing]
    for element in elements:
        assert element in advertised


@pytest.mark.launch(fixture=launch_description)
def test_driver_advertices_all_relevant_topics(launch_context, isolated, gripper_dummy):
    topic_list = [
        "/diagnostics",
        "/joint_states",
        "/state",
    ]
    check_each_in(topic_list, "get_topic_names_and_types")


@pytest.mark.launch(fixture=launch_description)
def test_driver_advertices_all_relevant_services(
    launch_context, isolated, gripper_dummy
):
    service_list = [
        "/acknowledge",
        "/brake_test",
        "/fast_stop",
        "/gripper_info",
        "/prepare_for_shutdown",
        "/reconnect",
        "/release_for_manual_movement",
        "/softreset",
        "/stop",
    ]
    check_each_in(service_list, "get_service_names_and_types")


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
