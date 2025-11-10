"""
Unit tests for workpiece handling functionality.

These tests cover:
- Basic gripping movements
- Gripping at expected positions (combined motion)
- Controlled and manual release operations

Depending on the connected hardware, additional modes may be tested:
- GPE mode (:= grip with brake for workpiece holding)
- Strong gripping mode (:= grip with force > 100% and GPE activated)
- Soft gripping mode (:= grip with controlled velocity)

All tests are designed to correctly handle workpieces in any gripping direction, if provided.
If no workpiece is provided, the tests perform empty grips and expect corresponding empty-grip outcomes.
"""
import time
from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import Scheduler
from schunk_gripper_library.tests.conftest import skip_if_no_drivers, get_workpiece_position
from concurrent.futures import wait

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


def read_float_param(driver: Driver, param_addr: str) -> float:
    read = driver.read_param
    decode = driver.decode_module_parameter
    values, value_type = decode(read(param_addr), param_addr)
    assert value_type == "float", f"Expected float type for param {param_addr}, got {value_type}"
    return values[0] 


def test_grip_and_release(drivers, scheduler, executor):    
    """ Tests basic grip and release functionality for all connected gripper drivers.
    Depending on the gripper variant and available features, different grip modes are tested:
    - EGK: basic grip and soft grip
    - EGU/EZU: basic grip and, if GPE available, strong grip
    All grips are tested with and without GPE (if available).
    """
    skip_if_no_drivers(drivers)

    futures = []
    for driver in drivers:
        def do_test():
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

        futures.append(executor.submit(do_test))

    wait(futures)


def grip_and_release_egk(driver, scheduler):
    """Issues basic and soft grip and release commands to an EGK driver.

    If there is workpiece position defined for the driver, grips are performed in the correct direction with
    WORKPIECE_GRIPPED as expected outcome, 
    otherwise grips are performed in both directions and NO_WORKPIECE_DETECTED is expected as outcome.
    """
    assert driver.get_variant() == "EGK", f"Driver is not an EGK variant: {driver.get_variant()}"
    gpe_available = driver.gpe_available()

    gripping_directions = get_gripping_directions(driver)
    if get_workpiece_position(driver) is not None:
        expected_grip_result = Driver.GripResult.WORKPIECE_GRIPPED
    else:
        expected_grip_result = Driver.GripResult.NO_WORKPIECE_DETECTED

    # basic grip force is defined to be in [50%..100%] 
    basic_forces_percent = [50, 75, 100]

    gpe_options = {gpe_available, False}

    # test basic grips
    for gpe_option in gpe_options:
        for force_percent in basic_forces_percent:
            for gripping_direction in gripping_directions:
                do_grip_cycle(driver, scheduler, force_percent, gripping_direction, gpe_option, expected_grip_result)

    # test soft grips
    for gpe_option in gpe_options:
        # compute velocities in [min_vel..max_grp_vel]
        min_vel = read_float_param(driver, Param.min_vel)
        max_grp_vel = read_float_param(driver, Param.max_grp_vel)  # in mm/s
        velocities_mms = [min_vel, (min_vel + max_grp_vel) / 2.0, max_grp_vel]  # in mm/s
        velocities_ums = [int(v * 1000) for v in velocities_mms]  # [mm/s] -> [um/s] (driver expects velocities in um/s)
        for velocity_ums in velocities_ums:
            for gripping_direction in gripping_directions:
                force = 100  # use max force for soft grip, otherwise the range of velocities has to be rescaled
                do_grip_cycle(driver, scheduler, force, gripping_direction, gpe_option, expected_grip_result, velocity_ums)
            

def grip_and_release_egu_ezu(driver, scheduler):
    """Issues basic and strong grip (if gpe available) and release commands to an EGU/EZU driver.

    If there is workpiece position defined for the driver, grips are performed in the correct direction with
    WORKPIECE_GRIPPED as expected outcome, 
    otherwise grips are performed in both directions and NO_WORKPIECE_DETECTED is expected as outcome.
    """
    assert driver.get_variant() in ["EGU", "EZU"], f"Driver is not an EGU/EZU variant: {driver.get_variant()}"
    gpe_available = driver.gpe_available()

    gripping_directions = get_gripping_directions(driver)
    if get_workpiece_position(driver) is not None:
        expected_grip_result = Driver.GripResult.WORKPIECE_GRIPPED
    else:
        expected_grip_result = Driver.GripResult.NO_WORKPIECE_DETECTED

    # basic grip force is defined to be in [50%..100%] 
    basic_forces_percent = [50, 75, 100]

    gpe_options = {gpe_available, False}
    # test basic grips
    for gpe_option in gpe_options:
        for force_percent in basic_forces_percent:
            for gripping_direction in gripping_directions:
                do_grip_cycle(driver, scheduler, force_percent, gripping_direction, gpe_option, expected_grip_result)
            
    if gpe_available:
        # test strong grips
        # compute max force percent based on max_grp_force [N] (:= 100%) and max_allow_force [N] (upper limit)
        max_grp_force = read_float_param(driver, Param.max_grp_force)
        max_allow_force = read_float_param(driver, Param.max_allow_force)
        max_force_percent = max_allow_force / max_grp_force * 100  # this is the upper limit for strong grips (>100%)
        strong_forces_percent = [101, (100 + max_force_percent) / 2.0, max_force_percent]
        strong_forces_percent = [int(x) for x in strong_forces_percent] # convert to int, as driver.grip expects int force percent
        for force_percent in strong_forces_percent:
            for gripping_direction in gripping_directions:
                do_grip_cycle(driver, scheduler, force_percent, gripping_direction, True, expected_grip_result)


def do_grip_cycle(driver: Driver, scheduler: Scheduler, force: int, outward: bool,
                               use_gpe: bool, expected_grip_result: Driver.GripResult, velocity_ums: int = 0):
    """Helper function that runs a grip-release-move cycle with the given parameters.
    Releasing is only performed if something was gripped.
    After grip and release, the gripper is moved back to the initial position.
    """
    actual_position_mm = driver.get_actual_position() / 1000.0  # [um] -> [mm]
    min_vel_mm = read_float_param(driver, Param.min_vel)
    max_vel_mm = read_float_param(driver, Param.max_vel)
    min_pos_mm = read_float_param(driver, Param.min_pos)  # min position in mm
    max_pos_mm = read_float_param(driver, Param.max_pos)  # max position in mm
    # in practice, if the gripper is fully opened or closed, 
    # the actual position might be slightly outside the allowed range, so we clip it
    home_position_mm = max(min_pos_mm, min(max_pos_mm, actual_position_mm))
    home_position_um = int(home_position_mm * 1000)
    # velocity for moving back to initial position between grips
    home_velocity_mm = (min_vel_mm + max_vel_mm) / 2.0
    home_velocity_um = int(home_velocity_mm * 1000)

    grip_result = driver.grip(force=force, velocity=velocity_ums, outward=outward, use_gpe=use_gpe, scheduler=scheduler)
    assert grip_result == expected_grip_result, f"Grip failed for force {force}% and velocity {velocity_ums} um/s. \
        Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
    if use_gpe:
        # give the break some time to settle
        time.sleep(0.5)
    # releasing is only allowed if something was gripped
    if grip_result == Driver.GripResult.WORKPIECE_GRIPPED or grip_result == Driver.GripResult.WRONG_WORKPIECE_GRIPPED:
        assert driver.release(scheduler=scheduler), f"Release after grip failed. Driver: {driver.addr_str}, \
            Status: {driver.get_status_diagnostics()}"
    # move back to initial position between grips
    move_success = driver.move_to_position(home_position_um, velocity=home_velocity_um, is_absolute=True, scheduler=scheduler)
    assert move_success,  f"Move to position failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"


def get_gripping_directions(driver: Driver) -> list[bool]:
    """Determines the gripping directions for gripping tests based on the workpiece definition (if any).

    If there is a workpiece defined for the driver, returns a single-element list with the correct gripping direction.
    The direction is determined from the defined workpiece position and the current finger position.

    If there is no workpiece defined, returns a two-element list with both gripping directions, 
    starting with the longer path.

    Returns:
        list[bool]: List of gripping directions to be used in tests.
                    True: grip from inside, False: grip from outside
    """
    directions = []  # True: grip from inside, False: grip from outside

    actual_position_mm = driver.get_actual_position() / 1000.0  # [um] -> [mm]
    min_pos_mm = read_float_param(driver, Param.min_pos)  # min position in mm
    max_pos_mm = read_float_param(driver, Param.max_pos)  # max position in mm

    if workpiece_position_mm := get_workpiece_position(driver):
        # workpiece is defined, proceed with determining gripping direction
        directions = [False]
        if actual_position_mm < workpiece_position_mm:
            # here the workpiece surrounds the fingers, so we need to grip from inside
            directions = [True]
    else:
        # no workpiece defined, grip in both directions and start with the longer path direction
        grip_from_inside_first = (actual_position_mm - min_pos_mm) < (max_pos_mm - actual_position_mm)
        if grip_from_inside_first:
            directions = [True, False]
        else:
            directions = [False, True]

    return directions
    
