from schunk_gripper_library.tests.conftest import skip_if_no_drivers

def test_acknowledge(drivers, scheduler):
    skip_if_no_drivers(drivers)

    for driver in drivers:
        acknowledged = driver.acknowledge(scheduler)
        assert(acknowledged), f"Failed to acknowledge driver. Status: {driver.get_status_diagnostics()}"

