#!/usr/bin/env python3
import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_pytest

from rclpy.node import Node
import time


@launch_pytest.fixture
def launch_description():
    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("schunk_egu_egk_gripper_driver"),
                "launch",
                "schunk.launch.py",
            ]
        )
    )
    return LaunchDescription([setup, launch_pytest.actions.ReadyToTest()])


@pytest.mark.launch(fixture=launch_description)
def test_normal_startup_works(launch_context, isolated):
    node = Node("test_startup")
    until_ready = 2.0  # sec
    time.sleep(until_ready)
    nodes = node.get_node_names()
    print(nodes)
    assert "schunk_gripper_driver" in nodes
