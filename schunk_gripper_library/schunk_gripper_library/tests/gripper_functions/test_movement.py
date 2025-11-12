"""Unit tests for gripper movement functions.

The tests cover:
    - Jogging (= tip mode)
    - Absolute positioning movement
    - Relative positioning movement
    - Controlled stop
    - Fast stop
"""

import time
from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import Scheduler
from schunk_gripper_library.tests.utils import skip_if_no_drivers, read_float_param, FloatParam


def test_jogging(drivers, scheduler, executor):
    """Tests jogging for all configured drivers.

    Tests cover:
        - Jogging in positive and negative direction
        - Different velocities (min, mid, max)
        - Using GPE and not using GPE (if available)
        - Repeated calls to start_jogging (idempotency)
    """
    skip_if_no_drivers(drivers)

    jog_duration_s = 1  # [s]
    futures = []
    for driver in drivers:
        gpe_available = driver.gpe_available()
        gpe_options = [False, True] if gpe_available else [False]

        def do_test():
            min_pos_mm = read_float_param(driver, FloatParam.min_pos)
            max_pos_mm = read_float_param(driver, FloatParam.max_pos)
            min_pos_um = int(min_pos_mm * 1000)
            max_pos_um = int(max_pos_mm * 1000)
            min_vel_mms = read_float_param(driver, FloatParam.min_vel)
            max_grp_vel_mms = read_float_param(driver, FloatParam.max_grp_vel)
            # velocity for moving back to initial position between grips
            home_velocity_mms = (min_vel_mms + max_grp_vel_mms) / 2.0
            home_velocity_ums = int(home_velocity_mms * 1000)

            jog_velocities_mms = [min_vel_mms, (min_vel_mms + max_grp_vel_mms) / 2.0, max_grp_vel_mms]
            jog_velocities_ums = [int(v * 1000) for v in jog_velocities_mms]  # [um/s], positive direction

            for use_gpe in gpe_options:
                for jog_velocity_ums in jog_velocities_ums:
                    actual_position_um = driver.get_actual_position()
                    actual_position_mm = actual_position_um / 1000.0
                    # in practice, if the gripper is fully opened or closed, 
                    # the actual position might be slightly outside the allowed range, so we clip it
                    home_position_mm = max(min_pos_mm, min(max_pos_mm, actual_position_mm))
                    home_position_um = int(home_position_mm * 1000)
                    
                    fingers_closer_to_center = (actual_position_um - min_pos_um) < (max_pos_um - actual_position_um)
                    if not fingers_closer_to_center:
                        # start jogging in negative direction (fingers moving towards center)
                        jog_velocity_ums *= -1

                    for _ in range(3):
                        # repeated calls to start jogging shall be idempotent
                        assert driver.start_jogging(velocity=jog_velocity_ums, scheduler=scheduler, use_gpe=use_gpe), \
                            f"Start jogging failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
                    time.sleep(jog_duration_s)
                    assert driver.stop_jogging(scheduler=scheduler), \
                        f"Stop jogging failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
                    # jog back
                    jog_velocity_ums *= -1
                    for _ in range(3):
                        # repeated calls to start jogging shall be idempotent
                        assert driver.start_jogging(velocity=jog_velocity_ums, scheduler=scheduler, use_gpe=use_gpe), \
                            f"Start jogging failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
                    time.sleep(jog_duration_s)
                    assert driver.stop_jogging(scheduler=scheduler), \
                        f"Stop jogging back failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
                    # move to original position to reset any position drift
                    move_success = driver.move_to_position(home_position_um, velocity=home_velocity_ums, is_absolute=True, scheduler=scheduler)
                    assert move_success,  f"Move to position failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
        
        futures.append(executor.submit(do_test))

    for future in futures:
        future.result()


def test_move_absolute(drivers, scheduler, executor):
    """Tests absolute positioning movement for all configured drivers.

    Tests cover:
        - Different positions (min, mid, max)
        - Different velocities (min, mid, max)
        - GPE usage (if available)
    """
    skip_if_no_drivers(drivers)

    futures = []
    for driver in drivers:
        def do_test():
            min_pos_mm = read_float_param(driver, FloatParam.min_pos)
            max_pos_mm = read_float_param(driver, FloatParam.max_pos)
            min_pos_um = int(min_pos_mm * 1000)
            max_pos_um = int(max_pos_mm * 1000)
            max_grp_vel_mms = read_float_param(driver, FloatParam.max_grp_vel)
            max_grp_vel_ums = int(max_grp_vel_mms * 1000)

            # Move to min position
            move_success = driver.move_to_position(position=min_pos_um, velocity=max_grp_vel_ums, is_absolute=True, scheduler=scheduler)
            assert move_success, f"Move to min position failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
            actual_position_um = driver.get_actual_position()
            assert abs(actual_position_um - min_pos_um) <= 1000,