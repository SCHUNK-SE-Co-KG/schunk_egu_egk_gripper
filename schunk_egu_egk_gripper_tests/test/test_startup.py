import pytest
from test.conftest import launch_description
from test.helpers import get_current_state
import time


@pytest.mark.launch(fixture=launch_description)
def test_driver_starts_in_ready_state(launch_context, isolated, gripper_dummy):
    time.sleep(2)
    assert get_current_state(variable="ready_for_operation") is True
