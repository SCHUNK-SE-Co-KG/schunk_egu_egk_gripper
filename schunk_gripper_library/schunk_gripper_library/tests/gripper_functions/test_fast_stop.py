from schunk_gripper_library.tests.conftest import skip_if_no_drivers


def test_fast_stop(drivers, scheduler):
    skip_if_no_drivers(drivers)

    for driver in drivers:
        assert driver.fast_stop(scheduler=scheduler), f"Failed to fast stop driver. Status: {driver.get_status_diagnostics()}"
