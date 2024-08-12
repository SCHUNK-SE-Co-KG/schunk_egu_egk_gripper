import pytest
from test.conftest import launch_description
from test.helpers import check_each_in, ActionReturnsResult
from schunk_egu_egk_gripper_interfaces.action import (  # type: ignore[attr-defined]
    MoveToAbsolutePosition,
    ReleaseWorkpiece,
)


@pytest.mark.launch(fixture=launch_description)
def test_driver_advertices_all_relevant_actions(running_driver):
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
def test_driver_moves_to_absolute_position(running_driver):
    goal = MoveToAbsolutePosition.Goal()
    action = ActionReturnsResult(
        "/move_to_absolute_position", MoveToAbsolutePosition, goal
    )
    action.event.wait(timeout=1)
    assert action.result.position_reached


@pytest.mark.launch(fixture=launch_description)
def test_driver_releases_workpieces(running_driver):
    goal = ReleaseWorkpiece.Goal()
    action = ActionReturnsResult("/release_workpiece", ReleaseWorkpiece, goal)
    action.event.wait(timeout=1)
    assert action.result.released_workpiece
