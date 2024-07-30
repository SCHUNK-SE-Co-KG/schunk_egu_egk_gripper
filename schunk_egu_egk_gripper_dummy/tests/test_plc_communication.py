from src.dummy import Dummy
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
    valid_bits = list(range(0, 10)) + [11, 12, 13, 14, 16, 17, 31]
    for bit in valid_bits:
        dummy.set_status_bit(bit=bit, value=True)
        result = dummy.get_status_bit(bit=bit)
        assert isinstance(result, int)  # successful calls get the bit's value
        assert result == 1


def test_dummy_only_touches_specified_bits():
    dummy = Dummy()
    before = dummy.get_plc_input()
    valid_bits = list(range(0, 10)) + [11, 12, 13, 14, 16, 17, 31]
    for bit in valid_bits:
        initial_value = dummy.get_status_bit(bit=bit)
        dummy.set_status_bit(bit=bit, value=True)
        dummy.set_status_bit(bit=bit, value=initial_value)

    assert dummy.get_plc_input() == before


# See p. 24 in
# Booting and establishing operational readiness [1]


@pytest.mark.skip()
def test_dummy_starts_in_error_state():
    dummy = Dummy()
    query = {"inst": dummy.plc_input, "count": 1}
    data = dummy.get_data(query)[0]
    assert data[0:2] == "80"
    assert data[30:] == "D9"  # ERR_FAST_STOP


@pytest.mark.skip()
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

    query = {"inst": dummy.plc_input, "count": 1}
    data = dummy.get_data(query)[0]
    assert data[0:2] == "11"
    assert data[30:] == "00"  # ERR_NONE
