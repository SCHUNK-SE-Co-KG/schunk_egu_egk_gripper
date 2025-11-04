from schunk_gripper_library.tests.conftest import skip_if_no_drivers


def test_stop(drivers, scheduler):
    skip_if_no_drivers(drivers)

    for driver in drivers:
        assert driver.stop(scheduler=scheduler, use_gpe=False), \
            f"Failed to stop driver. Status: {driver.get_status_diagnostics()}"
        if driver.gpe_available():
            assert driver.stop(scheduler=scheduler, use_gpe=True), \
                f"Failed to stop driver with GPE. Status: {driver.get_status_diagnostics()}"
