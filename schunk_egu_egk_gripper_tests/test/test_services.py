import pytest
from test.conftest import launch_description
from test.helpers import check_each_in
from schunk_egu_egk_gripper_interfaces.srv import (  # type: ignore[attr-defined]
    Acknowledge,
    BrakeTest,
    FastStop,
    Stop,
    ReleaseForManualMovement,
    GripperInfo,
    PrepareForShutdown,
    Softreset,
)
from std_srvs.srv import Trigger
from test.helpers import ServiceReturnsResult


@pytest.mark.launch(fixture=launch_description)
def test_driver_advertices_all_relevant_services(running_driver):
    service_list = [
        "/acknowledge",
        "/brake_test",
        "/fast_stop",
        "/gripper_info",
        "/prepare_for_shutdown",
        "/release_for_manual_movement",
        "/softreset",
        "/stop",
        "/grip",
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


@pytest.mark.launch(fixture=launch_description)
def test_driver_supports_gripper_info(running_driver):
    service = ServiceReturnsResult("/gripper_info", GripperInfo, GripperInfo.Request())
    service.event.wait(timeout=1)
    assert service.result.success is True
    assert service.result.info != ""


@pytest.mark.launch(fixture=launch_description)
def test_driver_supports_prepare_for_shutdown(running_driver):
    service = ServiceReturnsResult(
        "/prepare_for_shutdown", PrepareForShutdown, PrepareForShutdown.Request()
    )
    service.event.wait(timeout=1)
    assert service.result.success is True


@pytest.mark.launch(fixture=launch_description)
def test_driver_supports_release_for_manual_movement(running_driver):
    service = ServiceReturnsResult(
        "/release_for_manual_movement",
        ReleaseForManualMovement,
        ReleaseForManualMovement.Request(),
    )
    service.event.wait(timeout=1)
    assert service.result.success is True


@pytest.mark.launch(fixture=launch_description)
def test_driver_supports_softreset(running_driver):
    service = ServiceReturnsResult("/softreset", Softreset, Softreset.Request())
    service.event.wait(timeout=1)
    assert service.result.success is True


@pytest.mark.launch(fixture=launch_description)
def test_driver_supports_stop(running_driver):
    service = ServiceReturnsResult("/stop", Stop, Stop.Request())
    service.event.wait(timeout=1)
    assert service.result.success is True


@pytest.mark.launch(fixture=launch_description)
def test_driver_supports_grip(running_driver):
    service = ServiceReturnsResult("/grip", Trigger, Trigger.Request())
    service.event.wait(timeout=1)
    assert service.result.success is True
