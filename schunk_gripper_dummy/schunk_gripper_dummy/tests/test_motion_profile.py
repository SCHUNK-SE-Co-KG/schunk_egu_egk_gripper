from schunk_gripper_dummy.dummy import LinearMotion
import pytest
import time


def test_motion_initializes_with_initial_state_and_target_state():
    initial_pos = 0.0
    initial_speed = 0.0
    target_pos = 0.1
    target_speed = 55.1
    motion = LinearMotion(
        initial_pos=initial_pos,
        initial_speed=initial_speed,
        target_pos=target_pos,
        target_speed=target_speed,
    )
    assert pytest.approx(motion.initial_pos) == initial_pos
    assert pytest.approx(motion.initial_speed) == initial_speed
    assert pytest.approx(motion.target_pos) == target_pos
    assert pytest.approx(motion.target_speed) == target_speed


def test_motion_corrects_negative_or_zero_speed_arguments():
    initial_pos = 0
    initial_speed = -300
    target_pos = 0
    target_speed = 0
    motion = LinearMotion(
        initial_pos=initial_pos,
        initial_speed=initial_speed,
        target_pos=target_pos,
        target_speed=target_speed,
    )
    assert pytest.approx(motion.target_speed) == motion.min_speed
    assert pytest.approx(motion.initial_speed) == 0

    target_speed = -123
    motion = LinearMotion(
        initial_pos=initial_pos,
        initial_speed=initial_speed,
        target_pos=target_pos,
        target_speed=target_speed,
    )
    assert pytest.approx(motion.target_speed) == motion.min_speed


def test_motion_samples_current_state_at_given_time():
    initial_pos = 0
    initial_speed = 0
    target_pos = 130
    target_speed = 551
    motion = LinearMotion(
        initial_pos=initial_pos,
        initial_speed=initial_speed,
        target_pos=target_pos,
        target_speed=target_speed,
    )
    start = time.time()
    time.sleep(0.01)
    now = time.time() - start
    current_pos, current_speed = motion.sample(t=now)
    assert isinstance(current_pos, int)
    assert isinstance(current_speed, int)
    assert current_pos >= initial_pos
    assert current_pos <= target_pos
    assert current_speed >= initial_speed
    assert current_speed <= target_speed


def test_motion_samples_initial_state_at_time_less_or_equal_zero():
    initial_pos = 0
    initial_speed = 0.0
    target_pos = 100
    target_speed = 5510
    motion = LinearMotion(
        initial_pos=initial_pos,
        initial_speed=initial_speed,
        target_pos=target_pos,
        target_speed=target_speed,
    )
    current_pos, current_speed = motion.sample(t=-0.001)
    assert pytest.approx(current_pos) == motion.initial_pos
    assert pytest.approx(current_speed) == motion.initial_speed


def test_motion_samples_target_position_at_time_after_finish():
    initial_pos = 0
    initial_speed = 0
    target_pos = 100
    target_speed = 551
    motion = LinearMotion(
        initial_pos=initial_pos,
        initial_speed=initial_speed,
        target_pos=target_pos,
        target_speed=target_speed,
    )
    current_pos, current_speed = motion.sample(t=1000)
    assert pytest.approx(current_pos) == motion.target_pos
    assert pytest.approx(current_speed) == 0.0


def test_motion_supports_both_directions():
    initial_pos = 500
    initial_speed = 0
    target_speed = 5678

    # Positive direction
    target_pos = 1000
    motion = LinearMotion(
        initial_pos=initial_pos,
        initial_speed=initial_speed,
        target_pos=target_pos,
        target_speed=target_speed,
    )
    current_pos_1, _ = motion.sample(t=0.01)
    current_pos_2, _ = motion.sample(t=0.02)
    assert current_pos_1 < current_pos_2

    # Negative direction
    target_pos = 250
    motion = LinearMotion(
        initial_pos=initial_pos,
        initial_speed=initial_speed,
        target_pos=target_pos,
        target_speed=target_speed,
    )
    current_pos_1, _ = motion.sample(t=0.01)
    current_pos_2, _ = motion.sample(t=0.02)
    assert current_pos_1 > current_pos_2


def test_motion_estimates_finish_time():
    target_speed = 5678

    # Already finished
    motion = LinearMotion(
        initial_pos=1000,
        initial_speed=0,
        target_pos=1000,
        target_speed=target_speed,
    )
    assert pytest.approx(motion.time_finish) == 0.0

    # With different starts
    short = LinearMotion(
        initial_pos=900,
        initial_speed=0,
        target_pos=1000,
        target_speed=target_speed,
    ).time_finish
    middle = LinearMotion(
        initial_pos=500,
        initial_speed=0,
        target_pos=1000,
        target_speed=target_speed,
    ).time_finish
    long = LinearMotion(
        initial_pos=100,
        initial_speed=0,
        target_pos=1000,
        target_speed=target_speed,
    ).time_finish
    assert short < middle < long
