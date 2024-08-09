import pytest
from test.conftest import launch_description
from test.helpers import check_each_in
from schunk_egu_egk_gripper_interfaces.srv import (  # type: ignore[attr-defined]
    Acknowledge,
    BrakeTest,
    FastStop,
)
from test.helpers import ServiceReturnsResult


@pytest.mark.launch(fixture=launch_description)
def test_driver_advertices_all_relevant_services(running_driver):
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
def test_driver_is_ready_after_acknowledge(running_driver):
    service = ServiceReturnsResult("/acknowledge", Acknowledge, Acknowledge.Request())
    service.event.wait(timeout=1)
    assert service.result.success is True


@pytest.mark.launch(fixture=launch_description)
def test_driver_supports_break_test(running_driver):
    service = ServiceReturnsResult("/brake_test", BrakeTest, BrakeTest.Request())
    service.event.wait(timeout=1)
    assert service.result.success is True


@pytest.mark.launch(fixture=launch_description)
def test_driver_supports_fast_stop(running_driver):
    service = ServiceReturnsResult("/fast_stop", FastStop, FastStop.Request())
    service.event.wait(timeout=1)
    assert service.result.success is True
