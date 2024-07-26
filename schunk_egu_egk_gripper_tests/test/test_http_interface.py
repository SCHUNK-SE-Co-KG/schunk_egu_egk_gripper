import pytest
from test.conftest import launch_description
import time
from sensor_msgs.msg import JointState
from test.helpers import CheckTopic


@pytest.mark.launch(fixture=launch_description)
def test_driver_connnects_to_gripper_dummy(launch_context, isolated, gripper_dummy):
    until_dummy_ready = 3
    timeout = 3

    time.sleep(until_dummy_ready)
    assert CheckTopic("/joint_states", JointState).event.wait(timeout)
