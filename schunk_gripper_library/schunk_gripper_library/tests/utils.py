from schunk_gripper_library.tests.conftest import workpiece_position_map 
from schunk_gripper_library.driver import Driver
import pytest


class FloatParam:
    """Relevant module parameter addresses.
    """
    # gripping
    min_grp_force = "0x0658"  # [N]
    max_grp_force = "0x0660"  # [N]
    max_grp_force_sm = "0x06A0"  # not available for EGK, [N]
    max_allow_force = "0x06A8"  # not available for EGK, [N]
    max_grp_vel = "0x0650"  # [mm/s]
    # movement
    min_vel = "0x0628"  # [mm/s]
    max_vel = "0x0630"  # [mm/s]
    # positioning
    min_pos = "0x0600"  # [mm]
    max_pos = "0x0608"  # [mm]


def read_float_param(driver: Driver, param_addr: str) -> float:
    read = driver.read_param
    decode = driver.decode_module_parameter
    values, value_type = decode(read(param_addr), param_addr)
    assert value_type == "float", f"Expected float type for param {param_addr}, got {value_type}"
    return values[0] 


def get_workpiece_position(driver: Driver) -> int | None:
    """Returns the workpiece position for the given driver in [mm].
    """
    assert driver in workpiece_position_map, f"Driver at '{driver.addr_str}' not recognized."
    return workpiece_position_map[driver]


def skip_if_no_drivers(drivers):
    if not drivers:
        pytest.skip("No devices configured")