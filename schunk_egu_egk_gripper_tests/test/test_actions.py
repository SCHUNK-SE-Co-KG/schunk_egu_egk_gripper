import pytest
from test.conftest import launch_description
from test.helpers import check_each_in, ActionReturnsResult, get_current_state
from schunk_egu_egk_gripper_interfaces.action import (  # type: ignore[attr-defined]
    GripWithVelocity,
    GripWithPositionAndVelocity,
    MoveToAbsolutePosition,
    MoveToRelativePosition,
    ReleaseWorkpiece,
)
from control_msgs.action import GripperCommand


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
def test_driver_grips(running_driver):
    goal = (
        GripWithVelocity.Goal()
    )  # Our dummy is an EGK, which uses an additional velocity
    goal.gripping_force = 65.0  # % of max force
    goal.velocity_of_movement = 11.0
    goal.grip_direction = False  # close
    action = ActionReturnsResult("/grip", GripWithVelocity, goal)
    action.event.wait()
    assert action.result.workpiece_gripped
    assert action.result.no_workpiece_detected is False
    assert action.result.workpiece_lost is False


@pytest.mark.launch(fixture=launch_description)
def test_driver_grips_with_position(running_driver):
    goal = (
        GripWithPositionAndVelocity.Goal()
    )  # Our dummy is an EGK, which uses an additional velocity
    goal.absolute_position = 18.88
    goal.gripping_force = 65.0  # % of max force
    goal.velocity_of_movement = 13.0
    goal.grip_direction = False  # close
    action = ActionReturnsResult(
        "/grip_with_position", GripWithPositionAndVelocity, goal
    )
    action.event.wait()
    assert action.result.workpiece_gripped
    assert action.result.no_workpiece_detected is False
    assert action.result.workpiece_lost is False


@pytest.mark.launch(fixture=launch_description)
def test_driver_supports_gripper_control(running_driver):

    runs = [
        {"position": 0.0, "max_effort": 77.0},  # grip workpiece
        {"position": 82.4, "max_effort": 0.0},  # move to abs. position
        {"position": 66.3, "max_effort": 85.1},  # grip with position
    ]

    for run in runs:
        goal = GripperCommand.Goal()
        goal.command.position = run["position"]
        goal.command.max_effort = run["max_effort"]
        action = ActionReturnsResult("/gripper_control", GripperCommand, goal)
        action.event.wait()
        assert action.result.reached_goal
        assert action.result.stalled


@pytest.mark.launch(fixture=launch_description)
def test_driver_moves_to_absolute_position(running_driver):
    test_positions = [43.55, 17.02, 38.55, 103.7]
    test_speeds = [55.66, 10.5, 40.0, 88.8]
    for test_pos, test_speed in zip(test_positions, test_speeds):
        goal = MoveToAbsolutePosition.Goal()
        goal.absolute_position = test_pos
        goal.velocity_of_movement = test_speed
        action = ActionReturnsResult(
            "/move_to_absolute_position", MoveToAbsolutePosition, goal
        )
        action.event.wait()
        assert action.result.position_reached
        assert pytest.approx(action.result.absolute_position) == goal.absolute_position


@pytest.mark.launch(fixture=launch_description)
def test_driver_moves_to_relative_position(running_driver):
    initial_pos = get_current_state(variable="actual_position")
    goal = MoveToRelativePosition.Goal()
    goal.signed_relative_position = -10.03
    goal.velocity_of_movement = 15.0
    action = ActionReturnsResult(
        "/move_to_relative_position", MoveToRelativePosition, goal
    )
    action.event.wait()
    assert action.result.position_reached
    assert (
        pytest.approx(action.result.absolute_position)
        == initial_pos + goal.signed_relative_position
    )


@pytest.mark.launch(fixture=launch_description)
def test_driver_releases_workpieces(running_driver):
    goal = ReleaseWorkpiece.Goal()
    action = ActionReturnsResult("/release_workpiece", ReleaseWorkpiece, goal)
    action.event.wait()
    assert action.result.released_workpiece
