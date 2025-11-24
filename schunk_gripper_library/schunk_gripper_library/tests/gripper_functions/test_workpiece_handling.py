# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

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
from schunk_gripper_library.tests.utils.functions import skip_if_no_drivers, get_workpiece_position
from schunk_gripper_library.tests.utils.params import FloatParam, read_float_param


def test_grip_and_release(drivers):
    """ Tests basic grip and release for all connected gripper drivers.

    Depending on the gripper variant and available features, different grip modes are tested:
    - EGK: basic grip and soft grip
    - EGU/EZU: basic grip and, if GPE available, strong grip
    All grips are tested with and without GPE (if available).
    """
    skip_if_no_drivers(drivers)

    for driver in drivers:
        variant: str = driver.get_variant()  # str in ["EGU", "EZU", "EGK"]
        assert variant in ["EGU", "EZU", "EGK"], f"Unknown gripper variant: {variant}"

        if variant == "EGK":
            grip_and_release_egk(driver, False)
        elif variant in ["EGU", "EZU"]:
            grip_and_release_egu_ezu(driver)
        else:
            assert False, f"Unhandled gripper variant: {variant}"


def test_grip_and_release_at_expected_position(drivers):
    """Tests gripping at expected workpiece position for all connected gripper drivers.

    If a workpiece position is defined for the driver, the driver grips at that position and the
    test expects WORKPIECE_GRIPPED as outcome.
    If no workpiece position is defined, the driver is instructed to grip at some dynamically
    determined position and the test expects NO_WORKPIECE_DETECTED as outcome.

    Depending on the gripper variant and available features, different grip modes are tested:
    - EGK: basic grip and soft grip
    - EGU/EZU: basic grip and, if GPE available, strong grip
    All grips are tested with and without GPE (if available).
    """
    skip_if_no_drivers(drivers)

    for driver in drivers:
        variant: str = driver.get_variant()  # str in ["EGU", "EZU", "EGK"]
        assert variant in ["EGU", "EZU", "EGK"], f"Unknown gripper variant: {variant}"

        if variant == "EGK":
            grip_and_release_egk(driver, with_position=True)
        elif variant in ["EGU", "EZU"]:
            grip_and_release_egu_ezu(driver, with_position=True)
        else:
            assert False, f"Unhandled gripper variant: {variant}"


def test_manual_release(drivers):
    """Tests manual release after movement for all connected gripper drivers.
    """
    skip_if_no_drivers(drivers)

    for driver in drivers:
        # release for manual movement is only allowed by the firmware
        # if the gripper is in an error state => trigger an error first
        driver.fast_stop()
        # now the command shall succeed
        assert driver.release_for_manual_movement(), \
            f"Manual release failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
        time.sleep(0.5)
        # do a fast-stop to bring the gripper out of the manual release state (important for e. g. gripping afterwards)
        assert driver.fast_stop(), \
            f"Fast stop after manual release failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
        time.sleep(0.5)


def grip_and_release_egk(driver, with_position: bool = False):
    """Issues basic and soft grip and release commands to an EGK driver.

    If there is workpiece position defined for the driver,
    grips are performed in the correct direction with WORKPIECE_GRIPPED as expected outcome,
    otherwise grips are performed in both directions and NO_WORKPIECE_DETECTED is expected as outcome.

    @param with_position: If True, issued grip commands include the expected workpiece position.
    """
    assert driver.get_variant() == "EGK", f"Driver is not an EGK variant: {driver.get_variant()}"
    gpe_available = driver.gpe_available()

    gripping_direction = get_gripping_direction(driver)  # True: from inside, False: from outside
    if get_workpiece_position(driver) is not None:
        expected_grip_result = Driver.GripResult.WORKPIECE_GRIPPED
    else:
        expected_grip_result = Driver.GripResult.NO_WORKPIECE_DETECTED

    at_position_um = None
    if with_position:
        at_position_um = get_grip_expected_position(driver)

    # basic grip force is defined to be in [50%..100%]
    basic_forces_percent = [50, 75, 100]

    gpe_options = {gpe_available, False}
    # test basic grips
    for gpe_option in gpe_options:
        for force_percent in basic_forces_percent:
            do_grip_cycle(driver=driver, force=force_percent, outward=gripping_direction,
                          use_gpe=gpe_option, expected_grip_result=expected_grip_result, at_position_um=at_position_um)

    # test soft grips
    for gpe_option in gpe_options:
        # compute velocities in [min_vel..max_grp_vel]
        min_vel = read_float_param(driver, FloatParam.min_vel)
        max_grp_vel = read_float_param(driver, FloatParam.max_grp_vel)  # in mm/s
        velocities_mms = [min_vel, (min_vel + max_grp_vel) / 2.0, max_grp_vel]  # in mm/s
        velocities_ums = [int(v * 1000) for v in velocities_mms]  # [mm/s] -> [um/s] (driver expects velocities in um/s)
        for velocity_ums in velocities_ums:
            force = 100  # use max force for soft grip, otherwise the range of velocities has to be rescaled
            do_grip_cycle(driver=driver, force=force, outward=gripping_direction,
                          use_gpe=gpe_option, expected_grip_result=expected_grip_result, velocity_ums=velocity_ums,
                          at_position_um=at_position_um)


def grip_and_release_egu_ezu(driver, with_position: bool = False):
    """Issues basic and strong grip (if gpe available) and release commands to an EGU/EZU driver.

    If there is workpiece position defined for the driver,
    grips are performed in the correct direction with WORKPIECE_GRIPPED as expected outcome,
    otherwise grips are performed in both directions and NO_WORKPIECE_DETECTED is expected as outcome.
    """
    assert driver.get_variant() in ["EGU", "EZU"], f"Driver is not an EGU/EZU variant: {driver.get_variant()}"
    gpe_available = driver.gpe_available()

    gripping_direction = get_gripping_direction(driver)  # True: from inside, False: from outside
    if get_workpiece_position(driver) is not None:
        expected_grip_result = Driver.GripResult.WORKPIECE_GRIPPED
    else:
        expected_grip_result = Driver.GripResult.NO_WORKPIECE_DETECTED

    at_position_um = None
    if with_position:
        at_position_um = get_grip_expected_position(driver)

    # basic grip force is defined to be in [50%..100%]
    basic_forces_percent = [50, 75, 100]

    gpe_options = {gpe_available, False}
    # test basic grips
    for gpe_option in gpe_options:
        for force_percent in basic_forces_percent:
            do_grip_cycle(driver=driver, force=force_percent, outward=gripping_direction,
                          use_gpe=gpe_option, expected_grip_result=expected_grip_result, at_position_um=at_position_um)

    if gpe_available:
        # test strong grips
        # compute max force percent based on max_grp_force [N] (:= 100%) and max_allow_force [N] (upper limit)
        max_grp_force = read_float_param(driver, FloatParam.max_grp_force)
        max_allow_force = read_float_param(driver, FloatParam.max_allow_force)
        max_force_percent = max_allow_force / max_grp_force * 100  # this is the upper limit for strong grips (>100%)
        strong_forces_percent : list[int] = [101, int((100 + max_force_percent) / 2.0), int(max_force_percent)]
        for force_percent in strong_forces_percent:
            do_grip_cycle(driver=driver, force=force_percent, outward=gripping_direction,
                          use_gpe=gpe_option, expected_grip_result=expected_grip_result, at_position_um=at_position_um)


def do_grip_cycle(driver: Driver, force: int, outward: bool, use_gpe: bool, expected_grip_result: Driver.GripResult,
                  velocity_ums: int | None = None, at_position_um: int | None = None):
    """Helper function that runs a grip-release-move cycle with the given parameters.
    Releasing is only performed if something was gripped.
    After grip and release, the gripper is moved back to the initial position.
    """
    actual_position_mm = driver.get_actual_position() / 1000.0  # [um] -> [mm]
    min_vel_mms = read_float_param(driver, FloatParam.min_vel)
    max_vel_mms = read_float_param(driver, FloatParam.max_vel)
    min_pos_mm = read_float_param(driver, FloatParam.min_pos)  # min position in mm
    max_pos_mm = read_float_param(driver, FloatParam.max_pos)  # max position in mm
    # in practice, if the gripper is fully opened or closed,
    # the actual position might be slightly outside the allowed range, so we clip it
    home_position_mm = max(min_pos_mm, min(max_pos_mm, actual_position_mm))
    home_position_um = int(home_position_mm * 1000)
    # velocity for moving back to initial position between grips
    home_velocity_mms = (min_vel_mms + max_vel_mms) / 2.0
    home_velocity_ums = int(home_velocity_mms * 1000)

    time.sleep(0.5)
    grip_result = driver.grip(force=force, velocity=velocity_ums, position=at_position_um,
                              outward=outward, use_gpe=use_gpe)
    assert grip_result == expected_grip_result, f"Grip failed for force {force}% and velocity {velocity_ums} um/s. \
        Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"

    time.sleep(1)  # wait some time to allow for grip settling (especially when using GPE)

    # releasing is only allowed if something was gripped
    if grip_result == Driver.GripResult.WORKPIECE_GRIPPED or grip_result == Driver.GripResult.WRONG_WORKPIECE_GRIPPED:
        assert driver.release(), f"Release after grip failed. Driver: {driver.addr_str}, \
            Status: {driver.get_status_diagnostics()}"
        time.sleep(0.5)
    # move back to initial position between grips
    move_success = driver.move_to_position(home_position_um, velocity=home_velocity_ums, is_absolute=True)
    assert move_success, f"Move to position failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
    time.sleep(0.5)


def get_gripping_direction(driver: Driver) -> bool:
    """Determines the gripping direction for gripping tests based on the workpiece definition (if any).

    If there is a workpiece defined for the driver, returns the correct gripping direction.
    The direction is determined from the defined workpiece position and the current finger position.

    If there is no workpiece defined, returns the gripping direction with the longer path.

    Returns:
        bool: Gripping direction with
            True: grip from inside
            False: grip from outside
    """
    direction: bool = False

    actual_position_mm = driver.get_actual_position() / 1000.0  # [um] -> [mm]
    min_pos_mm = read_float_param(driver, FloatParam.min_pos)  # min position in mm
    max_pos_mm = read_float_param(driver, FloatParam.max_pos)  # max position in mm

    if workpiece_position_mm := get_workpiece_position(driver):
        # workpiece is defined, proceed with determining gripping direction
        direction = False  # from outside
        if actual_position_mm < workpiece_position_mm:
            # here the workpiece surrounds the fingers, so we need to grip from inside
            direction = True  # from inside
    else:
        # no workpiece defined, grip in both directions and start with the longer path direction
        grip_from_inside = (actual_position_mm - min_pos_mm) < (max_pos_mm - actual_position_mm)
        if grip_from_inside:
            direction = True  # from inside
        else:
            direction = False  # from outside

    return direction


def get_grip_expected_position(driver: Driver) -> int | None:
    """Determines the gripping position in [um] for gripping tests.

    If there is a workpiece defined, returns its position.
    Otherwise determines a dynamic gripping position between min/max position and current position.
    """
    grip_pos_um = 0
    wp_pos_mm = get_workpiece_position(driver)
    # if workpiece position is defined, use it, otherwise compute a dynamic position
    if wp_pos_mm is not None:
        grip_pos_um = int(wp_pos_mm * 1000)
    else:
        # compute a dynamic position between min/max pos and current pos
        actual_pos_mm = driver.get_actual_position() / 1000.0  # [um] -> [mm]
        min_pos_mm = read_float_param(driver, FloatParam.min_pos)  # min position in mm
        max_pos_mm = read_float_param(driver, FloatParam.max_pos)  # max position in mm
        grip_pos_mm : int = 0
        grip_from_inside = (actual_pos_mm - min_pos_mm) < (max_pos_mm - actual_pos_mm)
        if grip_from_inside:
            grip_pos_mm = int((actual_pos_mm + max_pos_mm) / 2.0)
        else:
            grip_pos_mm = int((actual_pos_mm + min_pos_mm) / 2.0)
        grip_pos_um = int(grip_pos_mm * 1000)

    return grip_pos_um
