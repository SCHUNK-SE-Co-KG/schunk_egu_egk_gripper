from src.dummy import Dummy

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


def test_dummy_only_touches_specified_status_bits():
    dummy = Dummy()
    before = dummy.get_plc_input()
    valid_bits = list(range(0, 10)) + [11, 12, 13, 14, 16, 17, 31]
    for bit in valid_bits:
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


def test_dummy_supports_reading_bits_in_plc_control():
    dummy = Dummy()
    valid_bits = list(range(0, 10)) + [11, 12, 13, 14, 16, 30, 31]
    for bit in valid_bits:
        result = dummy.get_control_bit(bit=bit)
        assert isinstance(result, int)  # successful calls get the bit's value


def test_dummy_rejects_reading_reserved_control_bits():
    dummy = Dummy()
    for bit in dummy.reserved_control_bits:
        assert isinstance(dummy.get_control_bit(bit), bool)  # call fails
        assert not dummy.get_control_bit(bit)
