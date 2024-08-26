from src.dummy import Dummy
import struct
import pytest

# [1]: https://stb.cloud.schunk.com/media/IM0046706.PDF


def test_dummy_initializes_plc_data_buffers():
    dummy = Dummy()
    assert dummy.data[dummy.plc_input][0] == dummy.plc_input_buffer.hex().upper()
    assert dummy.data[dummy.plc_output][0] == dummy.plc_output_buffer.hex().upper()


def test_dummy_returns_plc_data():
    dummy = Dummy()
    assert dummy.data[dummy.plc_input] == dummy.get_plc_input()
    assert dummy.data[dummy.plc_output] == dummy.get_plc_output()


def test_dummy_rejects_writing_reserved_status_bits():
    dummy = Dummy()
    invalid_bits = [-1, 999]
    for bit in invalid_bits + dummy.reserved_status_bits:
        assert not dummy.set_status_bit(bit, True)


def test_dummy_rejects_reading_reserved_status_bits():
    dummy = Dummy()
    for bit in dummy.reserved_status_bits:
        assert isinstance(dummy.get_status_bit(bit), bool)  # call fails
        assert not dummy.get_status_bit(bit)


def test_dummy_supports_reading_and_writing_bits_in_plc_status():
    dummy = Dummy()
    for bit in dummy.valid_status_bits:
        dummy.set_status_bit(bit=bit, value=True)
        result = dummy.get_status_bit(bit=bit)
        assert isinstance(result, int)  # successful calls get the bit's value
        assert result == 1


def test_dummy_only_touches_specified_status_bits():
    dummy = Dummy()
    before = dummy.get_plc_input()
    for bit in dummy.valid_status_bits:
        initial_value = dummy.get_status_bit(bit=bit)
        dummy.set_status_bit(bit=bit, value=True)
        dummy.set_status_bit(bit=bit, value=initial_value)

    assert dummy.get_plc_input() == before


def test_dummy_supports_reading_and_writing_status_error():
    dummy = Dummy()
    error_codes = ["AA", "bb", "0xcc"]
    expected = ["AA", "BB", "CC"]
    for error, expected in zip(error_codes, expected):
        dummy.set_status_error(error)
        assert dummy.get_status_error() == expected


def test_dummy_rejects_writing_invalid_status_error():
    dummy = Dummy()
    invalid_codes = ["zz", "-1", "aaa"]
    for error in invalid_codes:
        assert not dummy.set_status_error(error)


def test_dummy_supports_reading_and_writing_status_diagnostics():
    dummy = Dummy()
    diagnostics_code = "EF"
    dummy.set_status_diagnostics(diagnostics_code)
    assert dummy.get_status_diagnostics() == diagnostics_code


def test_dummy_rejects_writing_invalid_status_diagnostics():
    dummy = Dummy()
    invalid_codes = ["zz", "-1", "aaa"]
    for code in invalid_codes:
        assert not dummy.set_status_diagnostics(code)


def test_dummy_supports_reading_and_writing_bits_in_plc_control():
    dummy = Dummy()
    for bit in dummy.valid_control_bits:
        dummy.set_control_bit(bit=bit, value=True)
        result = dummy.get_control_bit(bit=bit)
        assert isinstance(result, int)  # successful calls get the bit's value
        assert result == 1


def test_dummy_rejects_reading_reserved_control_bits():
    dummy = Dummy()
    for bit in dummy.reserved_control_bits:
        assert isinstance(dummy.get_control_bit(bit), bool)  # call fails
        assert not dummy.get_control_bit(bit)


def test_dummy_rejects_writing_reserved_control_bits():
    dummy = Dummy()
    invalid_bits = [-1, 999]
    for bit in invalid_bits + dummy.reserved_control_bits:
        assert not dummy.set_control_bit(bit, True)


def test_dummy_supports_toggling_status_bits():
    dummy = Dummy()
    for bit in dummy.valid_status_bits:
        before = dummy.get_status_bit(bit)
        dummy.toggle_status_bit(bit=bit)
        after = dummy.get_status_bit(bit)
        assert after != before
        dummy.toggle_status_bit(bit=bit)
        assert dummy.get_status_bit(bit=bit) == before


def test_dummy_rejects_toggling_reserved_status_bits():
    dummy = Dummy()
    for bit in dummy.reserved_status_bits:
        assert not dummy.toggle_status_bit(bit)


def test_dummy_supports_reading_target_position():
    dummy = Dummy()
    target_pos = 12300  # um
    dummy.plc_output_buffer[4:8] = bytes(struct.pack("i", target_pos))
    assert pytest.approx(dummy.get_target_position(), rel=1e-3) == target_pos / 1000.0


def test_dummy_supports_reading_target_speed():
    dummy = Dummy()
    target_speed = 55300
    dummy.plc_output_buffer[8:12] = bytes(struct.pack("i", target_speed))
    assert pytest.approx(dummy.get_target_speed(), rel=1e-3) == target_speed / 1000.0


def test_dummy_supports_writing_actual_position():
    dummy = Dummy()
    pos = 0.123
    dummy.set_actual_position(pos)
    read_pos = dummy.data[dummy.actual_position][0]
    read_pos = struct.unpack("f", bytes.fromhex(read_pos))[0]
    assert pytest.approx(read_pos) == pos


def test_dummy_supports_writing_actual_speed():
    dummy = Dummy()
    speed = 66.5
    dummy.set_actual_speed(speed)
    read_speed = dummy.data[dummy.actual_speed][0]
    read_speed = struct.unpack("f", bytes.fromhex(read_speed))[0]
    assert pytest.approx(read_speed) == speed


def test_dummy_supports_reading_actual_position():
    dummy = Dummy()
    pos = 0.123
    dummy.set_actual_position(pos)
    assert pytest.approx(dummy.get_actual_position()) == pos


def test_dummy_supports_reading_actual_speed():
    dummy = Dummy()
    speed = 66.5
    dummy.set_actual_speed(speed)
    assert pytest.approx(dummy.get_actual_speed()) == speed


def test_dummy_supports_reading_and_writing_system_uptime():
    dummy = Dummy()
    uptime = 1234  # secs
    dummy.set_system_uptime(uptime)
    assert dummy.get_system_uptime() == uptime
