from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_visualization_nodes(context):
    hand_model = LaunchConfiguration('hand_model').perform(context)
    hand_side = LaunchConfiguration('hand_side').perform(context)

    if hand_model == 'g20' and hand_side != 'left':
        raise RuntimeError(
            'hand_model:=g20 currently supports only hand_side:=left because '
            'linkerhand-urdf does not include an authoritative G20 right-hand URDF.'
        )

    urdf_path = PathJoinSubstitution(
        [FindPackageShare('linker_hand_description'), 'robots', hand_model, f'{hand_model}.urdf.xacro']
    ).perform(context)
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('linker_hand_description'), 'rviz', 'linker_hand.rviz']
    ).perform(context)

    processed_doc = xacro.process_file(
        urdf_path,
        mappings={
            'ros2_control': 'false',
            'use_fake_hardware': 'false',
            'hand_side': hand_side,
            'can_interface': 'can0',
        },
    )
    robot_description = getattr(processed_doc, 'toprettyxml')(indent='  ')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('hand_model', default_value='l20', description='Linker hand model'),
            DeclareLaunchArgument('hand_side', default_value='left', description='Hand side: left or right'),
            OpaqueFunction(function=generate_visualization_nodes),
        ]
    )
