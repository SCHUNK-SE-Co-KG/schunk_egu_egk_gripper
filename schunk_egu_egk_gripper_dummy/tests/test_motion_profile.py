from src.dummy import LinearMotion
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
    initial_pos = 0.0
    initial_speed = -0.3
    target_pos = 0.1
    target_speed = 0.0
    motion = LinearMotion(
        initial_pos=initial_pos,
        initial_speed=initial_speed,
        target_pos=target_pos,
        target_speed=target_speed,
    )
    assert pytest.approx(motion.target_speed) == motion.min_speed
    assert pytest.approx(motion.initial_speed) == 0.0

    target_speed = -0.123
    motion = LinearMotion(
        initial_pos=initial_pos,
        initial_speed=initial_speed,
        target_pos=target_pos,
        target_speed=target_speed,
    )
    assert pytest.approx(motion.target_speed) == motion.min_speed


def test_motion_samples_current_state_at_given_time():
    initial_pos = 0.0
    initial_speed = 0.0
    target_pos = 1.3
    target_speed = 55.1
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
    assert isinstance(current_pos, float)
    assert isinstance(current_speed, float)
    assert current_pos >= initial_pos
    assert current_pos <= target_pos
    assert current_speed >= initial_speed
    assert current_speed <= target_speed


def test_motion_samples_initial_state_at_time_less_or_equal_zero():
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
    current_pos, current_speed = motion.sample(t=-0.001)
    assert pytest.approx(current_pos) == motion.initial_pos
    assert pytest.approx(current_speed) == motion.initial_speed


def test_motion_samples_target_position_at_time_after_finish():
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
    current_pos, current_speed = motion.sample(t=1000)
    assert pytest.approx(current_pos) == motion.target_pos
    assert pytest.approx(current_speed) == 0.0


def test_motion_supports_both_directions():
    initial_pos = 5.0
    initial_speed = 0.0
    target_speed = 5.678

    # Positive direction
    target_pos = 12.34
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
    target_pos = 1.234
    motion = LinearMotion(
        initial_pos=initial_pos,
        initial_speed=initial_speed,
        target_pos=target_pos,
        target_speed=target_speed,
    )
    current_pos_1, _ = motion.sample(t=0.01)
    current_pos_2, _ = motion.sample(t=0.02)
    assert current_pos_1 > current_pos_2
