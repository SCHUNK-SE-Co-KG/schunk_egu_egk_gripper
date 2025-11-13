"""Unit tests for additional gripper functions.

Tests cover:
    - Acknowledge
    - Brake Test
"""
from schunk_gripper_library.tests.utils import skip_if_no_drivers


def test_acknowledge(drivers, scheduler, executor):
    skip_if_no_drivers(drivers)

    futures = []
    for driver in drivers:
        def do_test():
            assert driver.acknowledge(scheduler), \
                f"Failed to acknowledge driver. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"

        futures.append(executor.submit(do_test))

    for future in futures:
        future.result()


def test_brake_test(drivers, scheduler, executor):
    skip_if_no_drivers(drivers)

    futures = []
    for driver in drivers:
        def do_test():
            if not driver.gpe_available():
                return  # brake test not available for this gripper

            # for brake test, the gripper must be standing still and not holding any workpiece.
            driver.release(scheduler)  # this may fail if no workpiece is held, but that's ok

            assert driver.brake_test(scheduler), \
                f"Failed to perform brake test. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"

        futures.append(executor.submit(do_test))

    for future in futures:
        future.result()