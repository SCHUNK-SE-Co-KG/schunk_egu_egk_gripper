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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ip = DeclareLaunchArgument(
        "IP",
        default_value="10.49.60.86",
        description="IP address of the gripper on your network",
    )
    port = DeclareLaunchArgument(
        "port",
        default_value="80",
        description="TCP/IP port of the gripper",
    )
    args = [ip, port]

    container = Node(
        name="gripper_container",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        emulate_tty=True,
    )

    load_composable_nodes = LoadComposableNodes(
        target_container="gripper_container",
        composable_node_descriptions=[
            ComposableNode(
                package="schunk_egu_egk_gripper_driver",
                plugin="SchunkGripperNode",
                name="schunk_gripper_driver",
                namespace="EGK_50_M_B",
                parameters=[
                    {"IP": LaunchConfiguration("IP")},
                    {"port": LaunchConfiguration("port")},
                    {"state_frq": 60.0},
                    {"rate": 10.0},
                    {"use_brk": False},
                    {"grp_pos_margin": 2.0},
                    {"grp_prepos_delta": 5.0},
                    {"zero_pos_ofs": 0.0},
                    {"grp_prehold_time": 0},
                    {"wp_release_delta": 5.0},
                    {"wp_lost_distance": 1.0},
                ],
            )
        ],
    )
    return launch.LaunchDescription(args + [container, load_composable_nodes])
