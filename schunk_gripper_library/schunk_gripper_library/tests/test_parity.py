from schunk_gripper_library.utility import supports_parity
import os


def test_parity_is_chosen_based_on_device_properties(pseudo_terminals):

    assert not supports_parity("non-existing-file")
    assert not supports_parity(pseudo_terminals[0])

    # Windows WSL setting.
    # COM[1-4] ports are mapped to /dev/ttyS[0-3] files
    serial_port = "/dev/ttyS0"
    if os.path.exists(serial_port):
        assert supports_parity(serial_port)
