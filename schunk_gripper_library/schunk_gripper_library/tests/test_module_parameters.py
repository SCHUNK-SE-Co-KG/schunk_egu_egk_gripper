from schunk_gripper_library.utility import skip_without_gripper
from schunk_gripper_library.driver import Driver
import struct


def test_driver_knows_readable_and_writable_module_parameters():
    driver = Driver()

    for params in [driver.readable_parameters, driver.writable_parameters]:
        # Check shape
        assert isinstance(params, dict)
        assert all(
            isinstance(key, str) and isinstance(value, dict)
            for key, value in params.items()
        )
        for value in params.values():
            assert "registers" in value and type(value["registers"]) is int
            assert "type" in value and type(value["type"]) is str


def test_driver_keeps_correct_order_when_decoding_arrays():
    driver = Driver()

    float_array = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    uint16_array = (301, 302, 303, 304, 305, 306)
    uint32_array = (100001, 100002, 100003, 100004, 100005, 100006)

    fake_parameters = {
        "0x1": {"type": "float[6]"},
        "0x2": {"type": "uint16[6]"},
        "0x3": {"type": "uint32[6]"},
    }
    driver.readable_parameters = fake_parameters
    driver.connected = True

    for endianness, fieldbus in zip(["<", ">"], ["", "PN"]):
        driver.fieldbus = fieldbus

        # Float arrays
        data = bytearray()
        for num in float_array:
            data.extend(struct.pack(f"{endianness}f", num))
        values, _ = driver.decode_module_parameter(data, param="0x1")
        assert values == float_array, f"endianness: {endianness}, fieldbus: {fieldbus}"

        # Uint16 arrays
        data = bytearray()
        for num in uint16_array:
            data.extend(struct.pack(f"{endianness}H", num))
        values, _ = driver.decode_module_parameter(data, param="0x2")
        assert values == uint16_array, f"endianness: {endianness}, fieldbus: {fieldbus}"

        # Uint32 arrays
        data = bytearray()
        for num in uint32_array:
            data.extend(struct.pack(f"{endianness}I", num))
        values, _ = driver.decode_module_parameter(data, param="0x3")
        assert values == uint32_array, f"endianness: {endianness}, fieldbus: {fieldbus}"


def test_driver_survives_encoding_invalid_module_parameters():
    driver = Driver()
    driver.connected = True

    valid_params = [
        "0x1330",  # bool
        "0x0528",  # float
        "0x11A8",  # uint8
        "0x0380",  # uint16
        "0x11A0",  # uint32
    ]
    invalid_data = [[1.0, 2.0], ["ok", "really?"], [42], [False, True, True]]

    # Sending invalid data to valid parameters shouldn't
    # crash the driver
    for param in valid_params:
        for data in invalid_data:
            driver.encode_module_parameter(data=data, param=param)
