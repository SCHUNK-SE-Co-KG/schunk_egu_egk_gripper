"""
Unit tests for workpiece handling.

Covers:
- Simple gripping movements
- Gripping at expected positions (combined motion)
- Controlled release and manual release preparation

Depending on the connected devices, tests may also include:
- GPE mode
- Strong gripping mode
- Soft gripping mode
"""
from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import Scheduler
from schunk_gripper_library.tests.conftest import skip_if_no_drivers

class Param:
    """Relevant module parameter addresses.
    """
    # gripping
    min_grp_force = "0x0658"
    max_grp_force = "0x0660"
    max_grp_force_sm = "0x06A0"  # not available for EGK
    max_allow_force = "0x06A8"  # not available for EGK
    max_grp_vel = "0x0650"
    # movement
    min_vel = "0x0628"
    max_vel = "0x0630"


def read_float_param(driver: Driver, scheduler: Scheduler, param_addr: str) -> float:
    read = driver.read_module_parameter
    decode = driver.decode_module_parameter
    values, value_type = decode(read(param_addr, scheduler), param_addr)
    assert value_type == "float", f"Expected float type for param {param_addr}, got {value_type}"
    return values[0] 

def test_grip_and_release(drivers, scheduler):    
    skip_if_no_drivers(drivers)

    for driver in drivers:
        variant: str = driver.get_variant()  # str in ["EGU", "EZU", "EGK"]
        assert variant in ["EGU", "EZU", "EGK"], f"Unknown gripper variant: {variant}"
        sub_variant: int = driver.get_sub_variant()
        gpe_available = driver.gpe_available()

        # Retrieve min/max forces and velocities from the driver for each gripper variant
        # and test all corner cases within these valid parameter ranges.

        # retrieve gripping params that are available for all variants
        min_grp_force = read_float_param(driver, scheduler, Param.min_grp_force)
        max_grp_force = read_float_param(driver, scheduler, Param.max_grp_force)

        if variant == "EGK":
            # EGK grip force is limited to the range [50%..100%]
            forces_percent = [50, 75, 100]
            # EGK can do soft grips, i. e. it supports velocity parameters
            max_grp_vel = read_float_param(driver, scheduler, Param.max_grp_vel)  # in mm/s
            velocities = [0.1 * max_grp_vel, 0.5 * max_grp_vel, max_grp_vel]  # in mm/s
            velocities *= 1000  # driver expects velocities in um/s
        else:
            if gpe_available:




            forces_percent = [50, 75, 100]
            if gpe_available:
                # EGU and EZU variants with GPE have extended force ranges, so we compute them here
                max_grp_force = read_float_param(driver, scheduler, Param.max_grp_force)
                max_allow_force = read_float_param(driver, scheduler, Param.max_allow_force)
                max_force_percent = max_allow_force / max_grp_force * 100
                forces_percent += [0.5 * max_force_percent, max_force_percent]


        max_grp_force_sm = 0
        max_grp_vel = 0
        if variant != "EGK":         
            max_grp_force_sm = read_float_param(driver, scheduler, Param.max_grp_force_sm)
        # movement params
        min_vel = read_float_param(driver, scheduler, Param.min_vel)
        max_vel = read_float_param(driver, scheduler, Param.max_vel)
        
        # Allowed forces as defined in the manual:
        # - All variants without GPE (brake): 50% <= force <= 100%
        # - Variants with GPE (brake):
        #       - EGU 50, 60, 80: 101% <= force <= 200%
        #       - EZU 30, 35, 40: 50% <= force <= 200%
        #       - EGU 70: 50% <= force <= 150%






    # driver = Driver()
    # for host, port, serial_port in zip(
    #     ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    # ):
    #     # Invalid arguments
    #     driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
    #     driver.acknowledge()
    #     max_int32 = 2147483647
    #     invalid_forces = [0.1, -0.1, 75.0, max_int32 + 1]
    #     for force in invalid_forces:
    #         assert driver.grip(force=force) == Driver.GripResult.ERROR

    #     driver.disconnect()