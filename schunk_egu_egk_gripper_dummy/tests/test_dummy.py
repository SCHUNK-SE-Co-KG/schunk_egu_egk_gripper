from src.dummy import Dummy
import pytest
import struct

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
    control_double_word = "04000000"
    set_position = "00000000"
    set_speed = "00000000"
    gripping_force = "00000000"
    command = {
        "inst": dummy.plc_output,
        "value": control_double_word + set_position + set_speed + gripping_force,
    }
    dummy.post(command)
    assert dummy.get_status_bit(0) == 1  # ready
    assert dummy.get_status_bit(7) == 0  # no error
    assert dummy.get_status_error() == "0"
    assert dummy.get_status_diagnostics() == "0"


def test_dummy_moves_to_absolute_position():
    dummy = Dummy()
    target_pos = 12.345
    target_speed = 50.3

    control_double_word = "00200000"
    set_position = bytes(struct.pack("f", target_pos)).hex().upper()
    set_speed = bytes(struct.pack("f", target_speed)).hex().upper()
    gripping_force = "00000000"
    command = {
        "inst": dummy.plc_output,
        "value": control_double_word + set_position + set_speed + gripping_force,
    }
    before = dummy.get_status_bit(bit=5)  # command received toggle

    # Motion
    dummy.post(command)

    # Done
    assert pytest.approx(dummy.get_actual_position()) == target_pos
    after = dummy.get_status_bit(bit=5)
    assert after != before
    assert dummy.get_status_bit(bit=13) == 1  # position reached
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed
