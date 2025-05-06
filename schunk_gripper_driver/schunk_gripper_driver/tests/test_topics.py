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

from schunk_gripper_library.utility import skip_without_gripper
from lifecycle_msgs.msg import Transition, State
from sensor_msgs.msg import JointState
import rclpy
import time
from functools import partial


@skip_without_gripper
def test_driver_advertises_state_depending_topics(lifecycle_interface):
    driver = lifecycle_interface
    gripper_topics = ["joint_states"]
    until_change_takes_effect = 0.1

    def exist(topics: list[str]) -> bool:
        existing = driver.node.get_topic_names_and_types()
        advertised = [i[0] for i in existing]
        for gripper in driver.list_grippers():
            for topic in topics:
                if f"/schunk/driver/{gripper}/{topic}" not in advertised:
                    return False
        return True

    for run in range(3):

        # After startup -> unconfigured
        driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)

        # After configure -> inactive
        driver.change_state(Transition.TRANSITION_CONFIGURE)
        time.sleep(until_change_takes_effect)
        assert not exist(gripper_topics)

        # After activate -> active
        driver.change_state(Transition.TRANSITION_ACTIVATE)
        time.sleep(until_change_takes_effect)
        assert exist(gripper_topics)

        # After deactivate -> inactive
        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        time.sleep(until_change_takes_effect)
        assert not exist(gripper_topics)

        # After cleanup -> unconfigured
        driver.change_state(Transition.TRANSITION_CLEANUP)
        time.sleep(until_change_takes_effect)


@skip_without_gripper
def test_driver_publishes_joint_states(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    grippers = driver.list_grippers()

    driver.change_state(Transition.TRANSITION_ACTIVATE)
    messages = []

    def check_fields(msg: JointState, messages: list[JointState]) -> None:
        messages.append(msg)
        assert msg.header.stamp
        assert msg.header.frame_id
        assert len(msg.name) >= 1
        assert len(msg.position) >= 1

    for gripper in grippers:
        _ = driver.node.create_subscription(
            JointState,
            f"/schunk/driver/{gripper}/joint_states",
            partial(check_fields, messages=messages),
            1,
        )

    for _ in range(10):
        rclpy.spin_once(driver.node)
        time.sleep(0.1)

    assert len(messages) >= 1
