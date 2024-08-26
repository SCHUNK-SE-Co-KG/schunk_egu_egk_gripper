#!/usr/bin/env python3
import pytest
from rclpy.node import Node
import time
from test.conftest import launch_description


@pytest.mark.launch(fixture=launch_description)
def test_normal_startup_works(launch_context, isolated):
    node = Node("test_startup")
    until_ready = 2.0  # sec
    time.sleep(until_ready)
    nodes = node.get_node_names()
    print(nodes)
    assert "schunk_gripper_driver" in nodes
