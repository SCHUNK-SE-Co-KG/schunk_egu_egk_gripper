from schunk_gripper_dummy.dummy import Dummy
import pytest
import struct
import time

# [1]: https://stb.cloud.schunk.com/media/IM0046706.PDF


def test_dummy_starts_a_background_thread():
    dummy = Dummy()
    assert not dummy.running
    dummy.start()
    assert dummy.running
    dummy.stop()
    assert not dummy.running


def test_dummy_survives_repeated_starts_and_stops():
    dummy = Dummy()
    for _ in range(3):
        dummy.start()
    assert dummy.running

    for _ in range(3):
        dummy.stop()
    assert not dummy.running


def test_dummy_reads_configuration_on_startup():
    dummy = Dummy()
    assert dummy.enum is not None
    assert dummy.data is not None
    assert dummy.metadata is not None


def test_dummy_starts_in_error_state():
    # See p. 24 in
    # Booting and establishing operational readiness [1]
    dummy = Dummy()
    assert dummy.get_status_bit(0) == 0  # not ready for operation
    assert dummy.get_status_bit(7) == 1  # there's an error
    assert dummy.get_status_error() == "D9"  # ERR_FAST_STOP
    assert dummy.get_status_diagnostics() == "EF"  # ERR_COMM_LOST


def test_dummy_is_ready_after_acknowledge():
    dummy = Dummy()
    dummy.set_control_bit(bit=2, value=True)
    dummy.process_control_bits()
    assert dummy.get_status_bit(0) == 1  # ready
    assert dummy.get_status_bit(7) == 0  # no error
    assert dummy.get_status_error() == "0"
    assert dummy.get_status_diagnostics() == "0"


def test_dummy_always_toggles_command_received_bit():
    dummy = Dummy()
    before = dummy.get_status_bit(bit=5)  # command received toggle
    dummy.process_control_bits()
    after = dummy.get_status_bit(bit=5)
    assert after != before


def test_dummy_moves_to_absolute_position():
    dummy = Dummy()
    target_positions = [12345, 10555, 77000, 1500]  # mu
    target_speeds = [50300, 40000, 10500, 20999]  # mu / s
    for target_pos, target_speed in zip(target_positions, target_speeds):
        control_double_word = "00200000"  # bit 13
        set_position = bytes(struct.pack("i", target_pos)).hex().upper()
        set_speed = bytes(struct.pack("i", target_speed)).hex().upper()
        gripping_force = "00000000"
        command = {
            "inst": dummy.plc_output,
            "value": control_double_word + set_position + set_speed + gripping_force,
        }

        dummy.post(command)
        assert dummy.get_actual_position() == pytest.approx(
            target_pos / 1000.0, rel=1e-3
        )
        assert dummy.get_status_bit(bit=13) == 1  # position reached
        assert dummy.get_status_bit(bit=4) == 1  # command successfully processed


def test_dummy_moves_to_relative_position():
    dummy = Dummy()
    target_pos = -5000  # mu
    target_speed = 12000  # mu / s
    control_double_word = "00400000"  # bit 14
    set_position = bytes(struct.pack("i", target_pos)).hex().upper()
    set_speed = bytes(struct.pack("i", target_speed)).hex().upper()
    gripping_force = "00000000"
    command = {
        "inst": dummy.plc_output,
        "value": control_double_word + set_position + set_speed + gripping_force,
    }
    before = dummy.get_actual_position()
    dummy.post(command)
    after = dummy.get_actual_position()
    assert after < before  # we are decreasing
    assert after == pytest.approx(before + target_pos / 1000.0, rel=1e-3)
    assert dummy.get_status_bit(bit=13) == 1  # position reached
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed


def test_dummy_updates_internal_state_when_moving():
    dummy = Dummy()
    query = {"offset": 15, "count": 3}  # actual position, speed, and current
    before = dummy.get_data(query)

    # Move
    target_pos = 10.34
    target_speed = 15.0
    assert pytest.approx(dummy.get_actual_position()) != target_pos
    dummy.move(target_pos=target_pos, target_speed=target_speed)
    assert pytest.approx(dummy.get_actual_position()) == target_pos

    after = dummy.get_data(query)
    assert before != after


def test_dummy_performs_break_test():
    dummy = Dummy()
    dummy.set_control_bit(bit=30, value=True)
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed


def test_dummy_performs_fast_stop():
    dummy = Dummy()
    dummy.set_control_bit(bit=0, value=False)  # fail-safe behavior
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=7) == 1  # error
    assert dummy.get_status_diagnostics() == "D9"  # ERR_FAST_STOP


def test_dummy_performs_controlled_stop():
    dummy = Dummy()
    dummy.set_control_bit(bit=1, value=True)
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=13) == 1  # position reached
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed


def test_dummy_supports_manual_release():
    dummy = Dummy()

    # Reject when not in error state
    dummy.set_status_bit(bit=7, value=False)  # clear error
    dummy.set_status_diagnostics("00")
    dummy.set_control_bit(bit=5, value=True)  # release
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=8) == 0  # not released

    # Accept when in error state
    dummy.set_control_bit(bit=0, value=False)  # trigger fast stop
    dummy.process_control_bits()
    dummy.set_control_bit(bit=5, value=True)  # release
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=8) == 1  # released for manual movement


def test_dummy_supports_prepare_for_shutdown():
    dummy = Dummy()
    dummy.set_control_bit(bit=3, value=True)
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=2) == 1  # ready for shutdown


def test_dummy_supports_release_workpiece():
    dummy = Dummy()
    dummy.set_control_bit(bit=11, value=True)
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed
    assert dummy.get_status_bit(bit=13) == 1  # position reached
    assert dummy.get_status_bit(bit=14) == 0  # pre-grip started
    assert dummy.get_status_bit(bit=12) == 0  # workpiece gripped
    assert dummy.get_status_bit(bit=17) == 0  # wrong workpiece gripped


def test_dummy_supports_softreset():
    dummy = Dummy()
    dummy.start()  # fake some system uptime
    initial = dummy.get_system_uptime()
    time.sleep(1.5)
    later = dummy.get_system_uptime()
    time.sleep(1.5)
    dummy.set_control_bit(bit=4, value=True)
    dummy.process_control_bits()
    after_reset = dummy.get_system_uptime()
    dummy.stop()

    assert initial < later
    assert after_reset < later  # restart resets the uptime


def test_dummy_supports_grip():
    dummy = Dummy()
    dummy.set_control_bit(bit=12, value=True)  # grip workpiece
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=12) == 1  # workpiece gripped
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed


def test_dummy_supports_grip_at_position():
    dummy = Dummy()
    dummy.set_control_bit(bit=16, value=True)  # grip workpiece at expected position
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=12) == 1  # workpiece gripped
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed
    assert dummy.get_status_bit(bit=31) == 1  # GPE activated
