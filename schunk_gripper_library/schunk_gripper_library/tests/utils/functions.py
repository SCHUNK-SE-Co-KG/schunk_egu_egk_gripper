from schunk_gripper_library.tests.conftest import workpiece_position_map
from schunk_gripper_library.driver import Driver
import pytest


def get_workpiece_position(driver: Driver) -> int | None:
    """Returns the workpiece position for the given driver in [mm].
    """
    assert driver in workpiece_position_map, f"Driver at '{driver.addr_str}' not recognized."
    return workpiece_position_map[driver]


def skip_if_no_drivers(drivers):
    if not drivers:
        pytest.skip("No devices configured")
