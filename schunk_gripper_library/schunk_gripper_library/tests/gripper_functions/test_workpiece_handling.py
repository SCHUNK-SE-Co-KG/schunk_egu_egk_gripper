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
from schunk_gripper_library.tests.conftest import skip_if_no_drivers, workpiece_at_position

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
    # positioning
    min_pos = "0x0600"
    max_pos = "0x0608"


def read_float_param(driver: Driver, scheduler: Scheduler, param_addr: str) -> float:
    read = driver.read_module_parameter
    decode = driver.decode_module_parameter
    values, value_type = decode(read(param_addr, scheduler), param_addr)
    assert value_type == "float", f"Expected float type for param {param_addr}, got {value_type}"
    return values[0] 

def test_grip_and_release(drivers, scheduler):    
    skip_if_no_drivers(drivers)

    # There are different modes of gripping and every gripper variant has its own capabilities.
    # According to the manual V53X:
    # - EGK:
    #     - Basic grip: grip with force in [50%..100%]
    #     - Soft grip: grip with velocity in [min_vel..max_grp_vel] and force in [50%..100%] as scaling factor.
    # - EGU/EZU without GPE:
    #     - Basic grip: grip with force in [50%..100%]
    # - EGU/EZU with GPE:
    #     - Basic grip: grip with force in [50%..100%]
    #     - Strong grip: grip with force >= 101% and uppper limit defined by max_allow_force

    for driver in drivers:
        variant: str = driver.get_variant()  # str in ["EGU", "EZU", "EGK"]
        assert variant in ["EGU", "EZU", "EGK"], f"Unknown gripper variant: {variant}"
        sub_variant: int = driver.get_sub_variant()
        gpe_available = driver.gpe_available()

        if variant == "EGK":
            grip_and_release_egk(driver, scheduler)
        elif variant in ["EGU", "EZU"]:
            grip_and_release_egu_ezu(driver, scheduler)
        else:
            assert False, f"Unhandled gripper variant: {variant}"

def grip_and_release_egk(driver, scheduler):
    """Issues basic and soft grip and release commands to an EGK driver.
    """
    assert driver.get_variant() == "EGK", f"Driver is not an EGK variant: {driver.get_variant()}"
    gpe_available = driver.gpe_available()
    # EGK grip force is defined to be in the range [50%..100%] (independent of GPE availability)
    forces_percent = [50, 75, 100]
    # determine gripping direction based on the current finger position and the workpiece position
    actual_position_mm = driver.get_actual_position() / 1000.0  # [um] -> [mm]
    workpiece_position_mm = workpiece_at_position(driver)  # [mm]
    grip_from_inside = False
    if actual_position_mm < workpiece_position_mm:
        # here the workpiece surrounds the fingers, so we need to grip from inside
        grip_from_inside = True

    gpe_options = {gpe_available, False}

    # test basic grips
    for gpe_option in gpe_options:
        for force_percent in forces_percent:
            grip_result = driver.grip(force=force_percent, outward=grip_from_inside, 
                                    use_gpe=gpe_option, scheduler=scheduler)
            assert grip_result == Driver.GripResult.WORKPIECE_GRIPPED, \
                f"EGK basic grip failed for force {force_percent}% and GPE set to {gpe_option}. \
                Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
            assert driver.release(scheduler=scheduler), f"EGK release failed. Driver: {driver.addr_str}, \
                Status: {driver.get_status_diagnostics()}"
            # move back to initial position between grips
            assert driver.move_to_position(actual_position_mm * 1000.0, is_absolute=True, scheduler=scheduler), \
                f"EGK move to position after release failed. Driver: {driver.addr_str}, \
                Status: {driver.get_status_diagnostics()}"
    
    # test soft grips
    for gpe_option in gpe_options:
        min_vel = read_float_param(driver, scheduler, Param.min_vel)
        max_grp_vel = read_float_param(driver, scheduler, Param.max_grp_vel)  # in mm/s
        velocities = [min_vel, (min_vel + max_grp_vel) / 2.0, max_grp_vel]  # in mm/s
        velocities *= 1000  # [mm/s] -> [um/s] (driver expects velocities in um/s)
        for velocity in velocities:
            force = 100  # use max force for soft grip, otherwise the range of velocities has to be rescaled
            grip_result = driver.grip(force=force, velocity=velocity, outward=grip_from_inside,
                                    use_gpe=gpe_option, scheduler=scheduler)
            assert grip_result == Driver.GripResult.WORKPIECE_GRIPPED, \
                f"EGK soft grip failed for velocity {velocity/1000.0} mm/s. Driver: {driver.addr_str}, \
                Status: {driver.get_status_diagnostics()}"
            assert driver.release(scheduler=scheduler), \
                f"EGK release after soft grip failed. Driver: {driver.addr_str}, \
                Status: {driver.get_status_diagnostics()}"
            # move back to initial position between grips
            assert driver.move_to_position(actual_position_mm * 1000.0, is_absolute=True, scheduler=scheduler), \
                f"EGK move to position after release failed. Driver: {driver.addr_str}, \
                Status: {driver.get_status_diagnostics()}"
            

def grip_and_release_egu_ezu(driver, scheduler):
    """Issues basic and strong grip (if gpe available) and release commands to an EGU/EZU driver.

    If there is workpiece position defined for the driver, grips are performed in the correct direction with
    WORKPIECE_GRIPPED as expected outcome, 
    otherwise grips are performed in both directions and NO_WORKPIECE_DETECTED is expected as outcome.
    """
    assert driver.get_variant() in ["EGU", "EZU"], f"Driver is not an EGU/EZU variant: {driver.get_variant()}"
    gpe_available = driver.gpe_available()
    actual_position_mm = driver.get_actual_position() / 1000.0  # [um] -> [mm]
    min_vel_mm = read_float_param(driver, scheduler, Param.min_vel)
    max_vel_mm = read_float_param(driver, scheduler, Param.max_vel)
    min_pos_mm = read_float_param(driver, scheduler, Param.min_pos)  # min position in mm
    max_pos_mm = read_float_param(driver, scheduler, Param.max_pos)  # max position in mm
    gripping_directions = []  # True: grip from inside, False: grip from outside

    # in practice, if the gripper is fully opened or closed, 
    # the actual position might be slightly outside the allowed range, so we clip it
    home_position_mm = max(min_pos_mm, min(max_pos_mm, actual_position_mm))
    # velocity for moving back to initial position between grips
    home_velocity_mm = (min_vel_mm + max_vel_mm) / 2.0

    # check if there is a workpiece position defined for this driver
    has_workpiece = workpiece_at_position(driver) is not None
    if has_workpiece:
        # workpiece is defined, proceed with determining gripping direction and expect WORKPIECE_GRIPPED as outcome
        workpiece_position_mm = workpiece_at_position(driver)
        gripping_directions = [False]
        if actual_position_mm < workpiece_position_mm:
            # here the workpiece surrounds the fingers, so we need to grip from inside
            gripping_directions = [True]
    else:
        # no workpiece defined, grip in both directions and expect NO_WORKPIECE_DETECTED as outcome
        # start with the longer path direction
        grip_from_inside_first = (actual_position_mm - min_pos_mm) < (max_pos_mm - actual_position_mm)
        if grip_from_inside_first:
            gripping_directions = [True, False]
        else:
            gripping_directions = [False, True]

    gpe_options = {gpe_available, False}
    # test basic grips
    for gpe_option in gpe_options:
        # basic grip force is defined to be in [50%..100%]
        forces_percent = [50, 75, 100]
        for force_percent in forces_percent:
            for gripping_direction in gripping_directions:
                print(f"Running basic grip (force: {force_percent}%, GPE: {gpe_option}, from inside: {gripping_direction})...")
                grip_result = driver.grip(force=force_percent, outward=gripping_direction, 
                                        use_gpe=gpe_option, scheduler=scheduler)
                expected_result = Driver.GripResult.WORKPIECE_GRIPPED if has_workpiece else Driver.GripResult.NO_WORKPIECE_DETECTED
                assert grip_result == expected_result, \
                    f"EGU/EZU basic grip failed (force: {force_percent}%, GPE: {gpe_option}, from inside: {gripping_direction}). \
                    Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
                # releasing is only allowed if something was gripped
                if grip_result == Driver.GripResult.WORKPIECE_GRIPPED or grip_result == Driver.GripResult.WRONG_WORKPIECE_GRIPPED:
                    print(f"Releasing grip...")
                    assert driver.release(scheduler=scheduler), f"EGU/EZU release failed. Driver: {driver.addr_str}, \
                        Status: {driver.get_status_diagnostics()}"
            # move back to initial position between grips
            print(f"Moving back to home position {home_position_mm} mm at velocity {home_velocity_mm} mm/s...")
            assert driver.move_to_position(int(home_position_mm * 1000), velocity=int(home_velocity_mm * 1000),
                                            is_absolute=True, scheduler=scheduler), \
                f"EGU/EZU move to position failed. Driver: {driver.addr_str}, \
                Status: {driver.get_status_diagnostics()}"
            
    if gpe_available:
        # test strong grips
        # compute max force percent based on max_grp_force [N] (:= 100%) and max_allow_force [N] (upper limit)
        max_grp_force = read_float_param(driver, scheduler, Param.max_grp_force)
        max_allow_force = read_float_param(driver, scheduler, Param.max_allow_force)
        max_force_percent = max_allow_force / max_grp_force * 100  # this is the upper limit for strong grips (>100%)
        strong_forces_percent = [101, (100 + max_force_percent) / 2.0, max_force_percent]
        for force_percent in strong_forces_percent:
            for gripping_direction in gripping_directions:
                print(f"Running strong grip (force: {force_percent}%, from inside: {gripping_direction})...")
                grip_result = driver.grip(force=force_percent, outward=gripping_direction, 
                                        use_gpe=True, scheduler=scheduler)
                expected_result = Driver.GripResult.WORKPIECE_GRIPPED if has_workpiece else Driver.GripResult.NO_WORKPIECE_DETECTED
                assert grip_result == expected_result, \
                    f"EGU/EZU strong grip failed for force {force_percent}%. \
                    Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
                # releasing is only allowed if something was gripped
                if grip_result == Driver.GripResult.WORKPIECE_GRIPPED or grip_result == Driver.GripResult.WRONG_WORKPIECE_GRIPPED:
                    print(f"Releasing grip...")
                    assert driver.release(scheduler=scheduler), f"EGU/EZU release after strong grip failed. Driver: {driver.addr_str}, \
                        Status: {driver.get_status_diagnostics()}"
            # move back to initial position between grips
            print(f"Moving back to home position {home_position_mm} mm at velocity {home_velocity_mm} mm/s...")
            assert driver.move_to_position(int(home_position_mm * 1000), velocity=int(home_velocity_mm * 1000),
                                            is_absolute=True, scheduler=scheduler), \
                f"EGU/EZU move to position failed. Driver: {driver.addr_str}, \
                Status: {driver.get_status_diagnostics()}"