from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='schunk_gripper',
            executable='schunk_gripper_driver',
            namespace='EGK_50_M_B',
            name='schunk_gripper_driver',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'IP': '10.49.60.91'},
                {'state_frq': 60.0},
                {'rate': 10.0},
                {'use_brk': False},
                {'grp_pos_margin': 2.0},
                {'grp_prepos_delta': 5.0},
                {'zero_pos_ofs': 0.0},
                {'grp_prehold_time': 0},
                {'wp_release_delta': 5.0},
                {'wp_lost_distance': 1.0},

            ],

        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui'
        )
    ])