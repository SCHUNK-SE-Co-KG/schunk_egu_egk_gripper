"""Unit tests for gripper movement functions.

The tests cover:
    - Controlled stop
    - Fast stop
    - Jogging (= tip mode)
    - Absolute positioning movement
    - Relative positioning movement
    - Jaw twitching
"""

import time
from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import Scheduler
from schunk_gripper_library.tests.utils import skip_if_no_drivers, read_float_param, FloatParam


def test_controlled_stop(drivers, scheduler, executor):
    skip_if_no_drivers(drivers)

    futures = []
    for driver in drivers:
        def do_test():
            assert driver.stop(scheduler=scheduler, use_gpe=False), \
            f"Failed to stop driver without GPE. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
            if driver.gpe_available():
                assert driver.stop(scheduler=scheduler, use_gpe=True), \
                f"Failed to stop driver with GPE. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
        
        futures.append(executor.submit(do_test))
    
    for future in futures:
        future.result()


def test_fast_stop(drivers, scheduler, executor):
    skip_if_no_drivers(drivers)

    futures = []
    for driver in drivers:
        def do_test():
            assert driver.fast_stop(scheduler=scheduler), \
            f"Failed to fast stop driver. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"

        futures.append(executor.submit(do_test))

    for future in futures:
        future.result()


def test_jogging(drivers, scheduler, executor):
    """Tests jogging for all connected drivers.

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
    """Tests absolute positioning movement for all connected drivers.

    Tests cover:
        - Different positions (min, mid, max)
        - Different velocities (min, mid, max)
        - GPE usage (if available)
    """
    skip_if_no_drivers(drivers)

    futures = []
    for driver in drivers:
        def do_test():
            actual_position_um = driver.get_actual_position()
            min_pos_um = int(read_float_param(driver, FloatParam.min_pos) * 1000)
            max_pos_um = int(read_float_param(driver, FloatParam.max_pos) * 1000)
            min_vel_ums = int(read_float_param(driver, FloatParam.min_vel) * 1000)
            max_vel_ums = int(read_float_param(driver, FloatParam.max_vel) * 1000)

            # homing params
            # velocity for moving back to initial position between grips
            home_velocity_ums = int((min_vel_ums + max_vel_ums) / 2.0)
             # in practice the actual position can exceed the allowed range => clip it
            home_position_um = int(max(min_pos_um, min(max_pos_um, actual_position_um)))
            
            positions_um = []
            fingers_closer_to_center = (actual_position_um - min_pos_um) < (max_pos_um - actual_position_um)
            if fingers_closer_to_center:
                # start with max position
                positions_um = [max_pos_um, (min_pos_um + max_pos_um) / 2, min_pos_um]
            else:
                # start with min position
                positions_um = [min_pos_um, (min_pos_um + max_pos_um) / 2, max_pos_um]
            positions_um = [int(p) for p in positions_um]  # the driver expects int inputs
            
            velocities_ums = [min_vel_ums, (min_vel_ums + max_vel_ums) / 2, max_vel_ums]
            velocities_ums = [int(v) for v in velocities_ums]
            
            gpe_available = driver.gpe_available()
            gpe_options = [False, True] if gpe_available else [False]

            for use_gpe in gpe_options:
                for velocity_ums in velocities_ums:
                    for position_um in positions_um:
                        move_success = driver.move_to_position(
                            position=position_um,
                            velocity=velocity_ums,
                            is_absolute=True,
                            scheduler=scheduler,
                            use_gpe=use_gpe
                        )
                        assert move_success, f"Move to position {position_um} failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
                        time.sleep(0.5)  # wait a bit between moves
            
            # move to original position
            move_success = driver.move_to_position(
                position=home_position_um,
                velocity=home_velocity_ums,
                is_absolute=True,
                scheduler=scheduler
            )
            assert move_success, f"Move to home position failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
            time.sleep(0.5)  # wait a bit after moving to home position
        
        futures.append(executor.submit(do_test))
    
    for future in futures:
        future.result()


def test_move_relative(drivers, scheduler, executor):
    """Tests relative positioning movement for all connected drivers.

    Tests cover:
        - Positive and negative movements
        - Different velocities (min, mid, max)
        - GPE usage (if available)
    """
    skip_if_no_drivers(drivers)

    futures = []
    for driver in drivers:
        def do_test():
            actual_position_um = driver.get_actual_position()
            min_pos_um = int(read_float_param(driver, FloatParam.min_pos) * 1000)
            max_pos_um = int(read_float_param(driver, FloatParam.max_pos) * 1000)
            min_vel_ums = int(read_float_param(driver, FloatParam.min_vel) * 1000)
            max_vel_ums = int(read_float_param(driver, FloatParam.max_vel) * 1000)

            # homing params
            # velocity for moving back to initial position between grips
            home_velocity_ums = int((min_vel_ums + max_vel_ums) / 2.0)
             # in practice the actual position can exceed the allowed range => clip it
            home_position_um = int(max(min_pos_um, min(max_pos_um, actual_position_um)))
            
            move_distance_um = int((max_pos_um - min_pos_um) / 4)  # move by quarter of full range
            move_directions = [1, -1]  # positive and negative directions
            
            velocities_ums = [min_vel_ums, (min_vel_ums + max_vel_ums) / 2, max_vel_ums]
            velocities_ums = [int(v) for v in velocities_ums]
            
            gpe_available = driver.gpe_available()
            gpe_options = [False, True] if gpe_available else [False]

            # Do the relative moves
            for use_gpe in gpe_options:
                for velocity_ums in velocities_ums:
                    for direction in move_directions:
                        # bring fingers to center position before starting relative movement
                        move_to_center(driver, scheduler)
                        time.sleep(0.5)
                        relative_distance_um = move_distance_um * direction
                        move_success = driver.move_to_position(
                            position=relative_distance_um,
                            velocity=velocity_ums,
                            is_absolute=False,
                            scheduler=scheduler,
                            use_gpe=use_gpe
                        )
                        assert move_success, f"Relative move by {relative_distance_um} um failed. \
                                Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
                        time.sleep(0.5)

            # after all relative moves, move to original position
            move_success = driver.move_to_position(
                position=home_position_um,
                velocity=home_velocity_ums,
                is_absolute=True,
                scheduler=scheduler
            )
            assert move_success, f"Move to home position failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
            time.sleep(0.5)

        futures.append(executor.submit(do_test))

    for future in futures:
        future.result()
            

def test_twitch_jaws(drivers, scheduler, executor):
    """Tests jaw twitching for all connected drivers.

    Tests cover:
        - Twitching from different starting positions (min, mid, max)
    """
    skip_if_no_drivers(drivers) # TODO: Test if gripper lands at the original position after twitching

    futures = []
    for driver in drivers:
        def do_test():
            actual_position_um = driver.get_actual_position()
            min_pos_um = int(read_float_param(driver, FloatParam.min_pos) * 1000)
            max_pos_um = int(read_float_param(driver, FloatParam.max_pos) * 1000)
            mid_pos_um = int((min_pos_um + max_pos_um) / 2)

            positions_um = [min_pos_um, mid_pos_um, max_pos_um]
            positions_um = [int(p) for p in positions_um]

            # homing params
            # velocity for moving back to initial position between grips
            home_velocity_ums = int((read_float_param(driver, FloatParam.min_vel) * 1000 + 
                                     read_float_param(driver, FloatParam.max_vel) * 1000) / 2.0)
            # clip to allowed range, as in practice actual position can exceed it
            home_position_um = max(min_pos_um, min(max_pos_um, actual_position_um)) 
            
            for position_um in positions_um:
                # move to starting position
                move_success = driver.move_to_position(
                    position=position_um,
                    velocity=home_velocity_ums,
                    is_absolute=True,
                    scheduler=scheduler
                )
                assert move_success, f"Move to twitch start position failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
                time.sleep(0.5)

                # perform twitching
                assert driver.twitch_jaws(scheduler=scheduler), \
                    f"Jaw twitching failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
                time.sleep(0.5)

            # move to original position
            move_success = driver.move_to_position(
                position=home_position_um,
                velocity=home_velocity_ums,
                is_absolute=True,
                scheduler=scheduler
            )
            assert move_success, f"Move to home position failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
            time.sleep(0.5)           

        futures.append(executor.submit(do_test))

    for future in futures:
        future.result()


def move_to_center(driver, scheduler):
    """Helper function to move gripper to center position."""
    min_pos_um = int(read_float_param(driver, FloatParam.min_pos) * 1000)
    max_pos_um = int(read_float_param(driver, FloatParam.max_pos) * 1000)
    min_vel_ums = int(read_float_param(driver, FloatParam.min_vel) * 1000)
    max_vel_ums = int(read_float_param(driver, FloatParam.max_vel) * 1000)
    center_pos_um = int((min_pos_um + max_pos_um) / 2)
    velocity_ums = int((min_vel_ums + max_vel_ums) / 2)
    move_success = driver.move_to_position(
        position=center_pos_um,
        velocity=velocity_ums,
        is_absolute=True,
        scheduler=scheduler
    )
    assert move_success, f"Move to center position failed. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"