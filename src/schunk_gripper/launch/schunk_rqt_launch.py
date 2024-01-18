import launch
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = Node(
        name='gripper_container',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        emulate_tty=True,
    )
            
    load_composable_nodes = LoadComposableNodes(
        target_container='gripper_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='schunk_gripper',
                    plugin='SchunkGripperNode',
                    name='schunk_gripper_driver',
                    namespace='EGK_50_M_B',
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
                            ]
                    )
            ],
    )
    
    rqt_launch = Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui'
    )

    return launch.LaunchDescription([container, rqt_launch, load_composable_nodes])