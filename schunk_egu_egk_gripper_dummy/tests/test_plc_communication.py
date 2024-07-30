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
