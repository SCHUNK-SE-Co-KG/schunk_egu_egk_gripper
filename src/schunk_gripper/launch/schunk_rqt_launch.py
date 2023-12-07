from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='schunk_gripper',
            executable='schunk_gripper_driver',
            name='schunk_gripper_driver',
            output='screen',
            parameters=[
                {'IP': '10.49.60.74'},
                {'state_frq': 60.0},
                {'rate': 10.0},
                {'Gripper Parameter.use_brk': False},
                {'Gripper Parameter.grp_pos_margin': 2.0},
                {'Gripper Parameter.grp_prepos_delta': 5.0},
                {'Gripper Parameter.zero_pos_ofs': 0.0},
                {'Gripper Parameter.grp_prehold_time': 0},
                {'Gripper Parameter.wp_release_delta': 5.0},
                {'Gripper Parameter.wp_lost_distance': 1.0},

            ],

        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui'
        )
    ])