"""Unit tests for shutdown and reset gripper functions.

Tests cover:
    - Prepare for shutdown
    - Soft reset
"""

from schunk_gripper_library.tests.utils import skip_if_no_drivers
from schunk_gripper_library.driver import Driver


def test_prepare_for_shutdown_and_soft_reset(drivers, scheduler, executor):
    """Tests prepare_for_shutdown and soft_reset functions for all connected drivers.
    
    Both functions are tested jointly, because soft_reset can only be called
    after prepare_for_shutdown.
    """
    skip_if_no_drivers(drivers)

    futures = []
    for driver in drivers:
        def do_test():
            assert driver.prepare_for_shutdown(scheduler), \
                f"Failed to prepare for shutdown. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
            assert driver.soft_reset(scheduler), \
                f"Failed to perform soft reset. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
        
        futures.append(executor.submit(do_test))

    for future in futures:
        future.result()
