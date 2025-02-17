# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

import pytest
import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from schunk_gripper_driver.test.conftest import (
    launch_description,
)
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from functools import partial


def change_state(node, client, transition_id):
    req = ChangeState.Request()
    req.transition.id = transition_id
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result()


def check_state(node, client, state_id):
    req = GetState.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result().current_state.id == state_id


@pytest.mark.launch(fixture=launch_description)
def test_driver_supports_repeated_configure_and_cleanup(isolated):
    node = Node("test_repeated_configure")
    timeout = Duration(seconds=2)

    change_state_client = node.create_client(ChangeState, "/schunk/driver/change_state")
    change_state_client.wait_for_service(timeout.nanoseconds / 1e9)
    get_state_client = node.create_client(GetState, "/schunk/driver/get_state")
    get_state_client.wait_for_service(timeout.nanoseconds / 1e9)

    in_state = partial(check_state, node, get_state_client)
    trigger = partial(change_state, node, change_state_client)

    for _ in range(3):
        trigger(Transition.TRANSITION_CONFIGURE)
        assert in_state(State.PRIMARY_STATE_INACTIVE)
        trigger(Transition.TRANSITION_CLEANUP)
        assert in_state(State.PRIMARY_STATE_UNCONFIGURED)


@pytest.mark.launch(fixture=launch_description)
def test_driver_supports_repeated_activate_and_deactivate(isolated):
    node = Node("test_repeated_activate")
    timeout = Duration(seconds=2)

    change_state_client = node.create_client(ChangeState, "/schunk/driver/change_state")
    change_state_client.wait_for_service(timeout.nanoseconds / 1e9)
    get_state_client = node.create_client(GetState, "/schunk/driver/get_state")
    get_state_client.wait_for_service(timeout.nanoseconds / 1e9)

    in_state = partial(check_state, node, get_state_client)
    trigger = partial(change_state, node, change_state_client)

    trigger(Transition.TRANSITION_CONFIGURE)

    for _ in range(3):
        trigger(Transition.TRANSITION_ACTIVATE)
        assert in_state(State.PRIMARY_STATE_ACTIVE)
        trigger(Transition.TRANSITION_DEACTIVATE)
        assert in_state(State.PRIMARY_STATE_INACTIVE)

    trigger(Transition.TRANSITION_CLEANUP)


@pytest.mark.launch(fixture=launch_description)
def test_primary_lifecycle_states(isolated):
    node = Node("test_primary_lifecycle_states")
    timeout = Duration(seconds=2)

    change_state_client = node.create_client(ChangeState, "/schunk/driver/change_state")
    change_state_client.wait_for_service(timeout.nanoseconds / 1e9)
    get_state_client = node.create_client(GetState, "/schunk/driver/get_state")
    get_state_client.wait_for_service(timeout.nanoseconds / 1e9)

    in_state = partial(check_state, node, get_state_client)
    trigger = partial(change_state, node, change_state_client)

    # Startup
    assert in_state(State.PRIMARY_STATE_UNCONFIGURED)

    # configure
    trigger(Transition.TRANSITION_CONFIGURE)
    assert in_state(State.PRIMARY_STATE_INACTIVE)

    # activate
    trigger(Transition.TRANSITION_ACTIVATE)
    assert in_state(State.PRIMARY_STATE_ACTIVE)

    # deactivate
    trigger(Transition.TRANSITION_DEACTIVATE)
    assert in_state(State.PRIMARY_STATE_INACTIVE)

    # cleanup
    trigger(Transition.TRANSITION_CLEANUP)
    assert in_state(State.PRIMARY_STATE_UNCONFIGURED)

    # shutdown
    trigger(Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)
    assert in_state(State.PRIMARY_STATE_FINALIZED)
