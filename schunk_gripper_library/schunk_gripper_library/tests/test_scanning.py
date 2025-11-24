from schunk_gripper_library.utility import EthernetScanner


def test_ethernet_scan(hms_chip):
    # The fixture provides an HMS chip that should
    # respond to the scanning requests.
    # Note: If this test is run on a machine with multiple network interfaces,
    # then the HMS chip will respond multiple times.
    with EthernetScanner() as scanner:
        grippers = scanner.scan()
        assert isinstance(grippers, list)
        assert len(grippers) >= 1

        for gripper in grippers:
            assert gripper["host"] != ""
            assert gripper["port"] == 80


def test_scan_raises_exception_without_context_manager():
    scanner = EthernetScanner()
    try:
        scanner.scan()
        assert False, "Expected RuntimeError"
    except RuntimeError:
        pass
