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
                {'rate': 10.0}
            ]
        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui'
        )
    ])