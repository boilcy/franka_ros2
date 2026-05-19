import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pnp_dual_fr3_bringup.launch_utils import (
    is_duo_config,
    load_yaml,
    parse_string_list,
    validate_arm_prefixes_unique,
    validate_duo_arrays_length,
)

import xacro


package_share = get_package_share_directory('pnp_dual_fr3_bringup')


def launch_arg_or_config(context, config, name, default):
    value = LaunchConfiguration(name).perform(context)
    if value != '':
        return value

    return str(config.get(name, default))


def generate_joint_state_nodes(context):
    robot_config_file = LaunchConfiguration('robot_config_file').perform(context)

    if not os.path.isabs(robot_config_file) and os.path.sep not in robot_config_file:
        robot_config_file = os.path.join(package_share, 'config', robot_config_file)

    configs = load_yaml(robot_config_file)
    config = next(iter(configs.values()))

    if not is_duo_config(config):
        raise RuntimeError(
            f'Configuration file {robot_config_file} does not contain a valid duo setup.'
        )

    robot_ips_str = launch_arg_or_config(context, config, 'robot_ips', config['robot_ips'])
    robot_types_str = launch_arg_or_config(context, config, 'robot_types', config['robot_types'])
    arm_prefixes_str = launch_arg_or_config(context, config, 'arm_prefixes', config['arm_prefixes'])
    use_fake_hardware_str = launch_arg_or_config(context, config, 'use_fake_hardware', 'false')
    fake_sensor_commands_str = launch_arg_or_config(context, config, 'fake_sensor_commands', 'true')
    load_gripper_str = launch_arg_or_config(context, config, 'load_gripper', 'false')
    namespace = launch_arg_or_config(context, config, 'namespace', '')
    joint_state_rate = int(launch_arg_or_config(context, config, 'joint_state_rate', 30))
    thread_priority_str = launch_arg_or_config(context, config, 'thread_priority', 50)

    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context)
    robot_types_list = parse_string_list(robot_types_str)
    robot_ips_list = parse_string_list(robot_ips_str)
    arm_prefixes_list = parse_string_list(arm_prefixes_str)

    validate_duo_arrays_length(robot_types_list, robot_ips_list, arm_prefixes_list)
    validate_arm_prefixes_unique(arm_prefixes_list)

    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare('pnp_dual_fr3_description'),
            'robots',
            'pnp_dual_fr3',
            'pnp_dual_fr3.urdf.xacro',
        ]
    ).perform(context)

    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            'ros2_control': 'true',
            'robot_types': robot_types_str,
            'robot_ips': robot_ips_str,
            'arm_prefixes': arm_prefixes_str,
            'hand': load_gripper_str,
            'use_fake_hardware': use_fake_hardware_str,
            'fake_sensor_commands': fake_sensor_commands_str,
            'is_async': 'true',
            'thread_priority': thread_priority_str,
        },
    ).toprettyxml(indent='  ')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'publish_robot_description': True},
            ],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[
                controllers_yaml,
                {'robot_description': robot_description},
                {'robot_types': robot_types_list},
                {'arm_prefixes': arm_prefixes_list},
            ],
            remappings=[('joint_states', 'franka/joint_states')],
            output={'stdout': 'screen', 'stderr': 'screen'},
            on_exit=Shutdown(),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[{'source_list': ['franka/joint_states'], 'rate': joint_state_rate}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_config_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('pnp_dual_fr3_bringup'), 'config', 'pnp_dual_fr3.config.yaml']
                ),
                description='PnP dual-arm robot configuration file.',
            ),
            DeclareLaunchArgument(
                'controllers_yaml',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('pnp_dual_fr3_bringup'),
                        'config',
                        'controllers.yaml',
                    ]
                ),
                description='ROS 2 control controller configuration file.',
            ),
            DeclareLaunchArgument(
                'robot_types',
                default_value='',
                description='Override robot_types from the YAML config, e.g. "[\'fr3v2\',\'fr3v2\']".',
            ),
            DeclareLaunchArgument(
                'robot_ips',
                default_value='',
                description='Override robot_ips from the YAML config.',
            ),
            DeclareLaunchArgument(
                'arm_prefixes',
                default_value='',
                description='Override arm_prefixes from the YAML config, e.g. "[\'left\',\'right\']".',
            ),
            DeclareLaunchArgument(
                'use_fake_hardware',
                default_value='',
                description='Override use_fake_hardware from the YAML config.',
            ),
            DeclareLaunchArgument(
                'fake_sensor_commands',
                default_value='',
                description='Override fake_sensor_commands from the YAML config.',
            ),
            DeclareLaunchArgument(
                'load_gripper',
                default_value='',
                description='Override load_gripper from the YAML config.',
            ),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Override namespace from the YAML config.',
            ),
            DeclareLaunchArgument(
                'joint_state_rate',
                default_value='',
                description='Override joint_state_rate from the YAML config.',
            ),
            DeclareLaunchArgument(
                'thread_priority',
                default_value='',
                description='Override thread_priority from the YAML config.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='',
                description='Accepted for command-line compatibility; this launch never starts RViz.',
            ),
            OpaqueFunction(function=generate_joint_state_nodes),
        ]
    )
