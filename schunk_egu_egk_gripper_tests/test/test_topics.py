import pytest
from test.conftest import launch_description
from test.helpers import check_each_in


@pytest.mark.launch(fixture=launch_description)
def test_driver_advertices_all_relevant_topics(launch_context, isolated, gripper_dummy):
    topic_list = [
        "/diagnostics",
        "/joint_states",
        "/state",
    ]
    check_each_in(topic_list, "get_topic_names_and_types")
