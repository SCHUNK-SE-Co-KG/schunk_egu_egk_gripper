import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_pytest
import subprocess
from ament_index_python.packages import get_package_share_directory


# We avoid black's F811, F401 linting warnings
# by using pytest's special conftest.py file.
# See documentation here:
# https://docs.pytest.org/en/7.1.x/reference/fixtures.html#conftest-py-sharing-fixtures-across-multiple-files  # noqa: E501


@pytest.fixture()
def isolated():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def gripper_dummy():
    package_name = "schunk_egu_egk_gripper_dummy"
    dummy_dir = get_package_share_directory(package_name)
    start_cmd = " uvicorn main:server --port 8000"
    p = subprocess.Popen(
        "exec" + start_cmd, stdin=subprocess.PIPE, cwd=dummy_dir, shell=True
    )

    print("Started gripper dummy")
    yield
    p.kill()
    print("Stopped gripper dummy")


@launch_pytest.fixture
def launch_description():
    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("schunk_egu_egk_gripper_driver"),
                "launch",
                "schunk.launch.py",
            ]
        ),
        launch_arguments={
            "IP": "127.0.0.1",
            "port": "8000",
        }.items(),
    )
    return LaunchDescription([setup, launch_pytest.actions.ReadyToTest()])
