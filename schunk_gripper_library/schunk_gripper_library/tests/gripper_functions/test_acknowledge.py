from schunk_gripper_library.tests.conftest import skip_if_no_drivers

def test_acknowledge(drivers, scheduler):
    skip_if_no_drivers(drivers)

    for driver in drivers:
        assert driver.acknowledge(scheduler), \
        f"Failed to acknowledge driver. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
