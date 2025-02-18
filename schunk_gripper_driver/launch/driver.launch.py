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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


port = DeclareLaunchArgument(
    "port",
    default_value="/dev/pts/13",
    description="The gripper's serial port",
)
args = [port]


def generate_launch_description():
    return LaunchDescription(
        args
        + [
            Node(
                package="schunk_gripper_driver",
                namespace="schunk",
                executable="driver.py",
                name="driver",
                parameters=[
                    {"port": LaunchConfiguration("port")},
                ],
            )
        ]
    )
