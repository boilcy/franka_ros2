from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_robot_nodes(context):
    hand_model = LaunchConfiguration('hand_model').perform(context)
    hand_side = LaunchConfiguration('hand_side').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    can_interface = LaunchConfiguration('can_interface').perform(context)
    can_baudrate = LaunchConfiguration('can_baudrate').perform(context)

    if hand_model == 'g20' and hand_side != 'left':
        raise RuntimeError(
            'hand_model:=g20 currently supports only hand_side:=left because '
            'linkerhand-urdf does not include an authoritative G20 right-hand URDF.'
        )

    urdf_path = PathJoinSubstitution(
        [FindPackageShare('linker_hand_description'), 'robots', hand_model, f'{hand_model}.urdf.xacro']
    ).perform(context)

    processed_doc = xacro.process_file(
        urdf_path,
        mappings={
            'ros2_control': 'true',
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware').perform(context),
            'hand_side': hand_side,
            'can_interface': can_interface,
            'can_baudrate': can_baudrate,
        },
    )
    robot_description = getattr(processed_doc, 'toprettyxml')(indent='  ')

    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context)
    if not controllers_yaml:
        controllers_yaml = PathJoinSubstitution(
            [FindPackageShare('linker_hand_bringup'), 'config', f'controllers_{hand_model}.yaml']
        ).perform(context)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[controllers_yaml, {'robot_description': robot_description}],
            output='screen',
            on_exit=Shutdown(),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['hand_controller'],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('hand_model', default_value='l20', description='Linker hand model'),
            DeclareLaunchArgument('hand_side', default_value='left', description='Hand side: left or right'),
            DeclareLaunchArgument('namespace', default_value='', description='ROS namespace'),
            DeclareLaunchArgument('can_interface', default_value='can0', description='Linux SocketCAN interface'),
            DeclareLaunchArgument('can_baudrate', default_value='1000000', description='CAN bitrate'),
            DeclareLaunchArgument('use_fake_hardware', default_value='false', description='Use fake hardware'),
            DeclareLaunchArgument(
                'controllers_yaml',
                default_value='',
                description='Override the default controllers YAML file.',
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
