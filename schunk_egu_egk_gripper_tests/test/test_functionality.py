import pytest
from test.conftest import launch_description
from test.helpers import get_current_state


@pytest.mark.launch(fixture=launch_description)
def test_driver_starts_in_not_ready_state(launch_context, isolated, gripper_dummy):
    assert get_current_state(variable="ready_for_operation") is False
