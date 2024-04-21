# Copyright 2024 SCHUNK SE & Co. KG
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

import launch
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = Node(
        name="gripper_container",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        emulate_tty=True,
    )
    rqt_launch = Node(package="rqt_gui", executable="rqt_gui", name="rqt_gui")
    load_composable_nodes = LoadComposableNodes(
        target_container="gripper_container",
        composable_node_descriptions=[
            ComposableNode(
                package="schunk_gripper",
                plugin="SchunkGripperNode",
                name="schunk_gripper_driver",
                namespace="EGK_50_M_B",
                parameters=[
                    {"state_frq": 10.0},
                    {"rate": 10.0},
                    {"IP": "10.49.60.86"},
                    {"Gripper_Parameter.use_brk": False},
                    {"Gripper_Parameter.grp_pos_margin": 2.0},
                    {"Gripper_Parameter.grp_prepos_delta": 5.0},
                    {"Gripper_Parameter.zero_pos_ofs": 0.0},
                    {"Gripper_Parameter.grp_prehold_time": 0},
                    {"Gripper_Parameter.wp_release_delta": 5.0},
                    {"Gripper_Parameter.wp_lost_distance": 1.0},
                ],
            )
        ],
    )

    return launch.LaunchDescription([container, load_composable_nodes, rqt_launch])
