# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

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
