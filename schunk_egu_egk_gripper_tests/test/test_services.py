import pytest
from test.conftest import launch_description
from test.helpers import check_each_in
from schunk_egu_egk_gripper_interfaces.srv import (  # type: ignore[attr-defined]
    Acknowledge,
)
from test.helpers import ServiceReturnsResult


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
def test_driver_is_ready_after_acknowledge(launch_context, isolated, gripper_dummy):
    timeout = 3
    service = ServiceReturnsResult("/acknowledge", Acknowledge, Acknowledge.Request())
    service.event.wait(timeout)
    assert service.result.success is True
