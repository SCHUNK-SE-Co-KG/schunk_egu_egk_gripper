"""Unit tests for additional gripper functions.

Tests cover:
    - Acknowledge
    - Brake Test
"""
from schunk_gripper_library.tests.utils.functions import skip_if_no_drivers


def test_acknowledge(drivers):
    skip_if_no_drivers(drivers)

    for driver in drivers:
        assert driver.acknowledge(), \
            f"Failed to acknowledge driver. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"


def test_brake_test(drivers):
    skip_if_no_drivers(drivers)

    for driver in drivers:
        if not driver.gpe_available():
            return  # brake test not available for this gripper

        # for brake test, the gripper must be standing still and not holding any workpiece.
        driver.release()  # this may fail if no workpiece is held, but that's ok

        assert driver.brake_test(), \
            f"Failed to perform brake test. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
