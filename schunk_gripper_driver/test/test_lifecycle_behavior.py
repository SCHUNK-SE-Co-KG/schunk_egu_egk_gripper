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
from schunk_gripper_library.tests.conftest import skip_without_gripper
from lifecycle_msgs.msg import Transition, State


@skip_without_gripper
def test_driver_supports_repeated_configure_and_cleanup(lifecycle_interface):
    driver = lifecycle_interface
    for _ in range(3):
        for protocol in ["modbus", "tcpip"]:
            driver.use_protocol(protocol)
            driver.change_state(Transition.TRANSITION_CONFIGURE)
            assert driver.check_state(State.PRIMARY_STATE_INACTIVE)
            driver.change_state(Transition.TRANSITION_CLEANUP)
            assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)


@skip_without_gripper
def test_driver_supports_repeated_activate_and_deactivate(lifecycle_interface):
    driver = lifecycle_interface
    for protocol in ["modbus", "tcpip"]:
        driver.use_protocol(protocol)
        driver.change_state(Transition.TRANSITION_CONFIGURE)
        for _ in range(3):
            driver.change_state(Transition.TRANSITION_ACTIVATE)
            assert driver.check_state(State.PRIMARY_STATE_ACTIVE)
            driver.change_state(Transition.TRANSITION_DEACTIVATE)
            assert driver.check_state(State.PRIMARY_STATE_INACTIVE)
        driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_primary_lifecycle_states(lifecycle_interface):
    driver = lifecycle_interface
    for protocol in ["modbus", "tcpip"]:
        driver.use_protocol(protocol)

        # Startup
        assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)

        # configure
        driver.change_state(Transition.TRANSITION_CONFIGURE)
        assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

        # activate
        driver.change_state(Transition.TRANSITION_ACTIVATE)
        assert driver.check_state(State.PRIMARY_STATE_ACTIVE)

        # deactivate
        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

        # cleanup
        driver.change_state(Transition.TRANSITION_CLEANUP)
        assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)
