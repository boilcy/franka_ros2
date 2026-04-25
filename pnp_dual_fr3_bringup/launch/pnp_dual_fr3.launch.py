import os

from ament_index_python.packages import get_package_share_directory

import franka_bringup.launch_utils as launch_utils

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro
import yaml


package_share = get_package_share_directory('pnp_dual_fr3_bringup')

load_yaml = launch_utils.load_yaml
parse_string_list = launch_utils.parse_string_list
validate_duo_arrays_length = launch_utils.validate_duo_arrays_length
validate_arm_prefixes_unique = launch_utils.validate_arm_prefixes_unique
is_duo_config = launch_utils.is_duo_config


def load_yaml_from_package(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, relative_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_robot_nodes(context):
    robot_config_file = LaunchConfiguration('robot_config_file').perform(context)

    if not os.path.isabs(robot_config_file) and os.path.sep not in robot_config_file:
        robot_config_file = os.path.join(package_share, 'config', robot_config_file)

    configs = load_yaml(robot_config_file)
    config = next(iter(configs.values()))

    if not is_duo_config(config):
        raise RuntimeError(
            f'Configuration file {robot_config_file} does not contain a valid duo setup.'
        )

    robot_ips_str = str(config['robot_ips'])
    robot_types_str = str(config['robot_types'])
    arm_prefixes_str = str(config['arm_prefixes'])
    use_fake_hardware_str = str(config.get('use_fake_hardware', 'true'))
    fake_sensor_commands_str = str(config.get('fake_sensor_commands', 'true'))
    load_gripper_str = str(config.get('load_gripper', 'false'))
    namespace = str(config.get('namespace', ''))
    joint_state_rate = int(config.get('joint_state_rate', 30))
    use_rviz = str(config.get('use_rviz', 'true')).lower() == 'true'
    thread_priority_str = str(config.get('thread_priority', 50))

    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

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

    srdf_path = PathJoinSubstitution(
        [
            FindPackageShare('pnp_dual_fr3_moveit_config'),
            'config',
            'pnp_dual_fr3.srdf.xacro',
        ]
    ).perform(context)

    robot_description_semantic = xacro.process_file(
        srdf_path,
        mappings={
            'robot_types': robot_types_str,
            'arm_prefixes': arm_prefixes_str,
            'hand': load_gripper_str,
        },
    ).toprettyxml(indent='  ')

    kinematics_config = {
        'robot_description_kinematics': load_yaml_from_package(
            'pnp_dual_fr3_moveit_config', 'config/kinematics.yaml'
        )
    }
    joint_limits_config = {
        'robot_description_planning': load_yaml_from_package(
            'pnp_dual_fr3_moveit_config', 'config/pnp_dual_fr3_joint_limits.yaml'
        )
    }

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(
        load_yaml_from_package('pnp_dual_fr3_moveit_config', 'config/ompl_planning.yaml')
    )

    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml_from_package(
            'pnp_dual_fr3_moveit_config', 'config/pnp_dual_fr3_controllers.yaml'
        ),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_description}],
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
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['left_arm_controller', '--controller-manager-timeout', '60'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['right_arm_controller', '--controller-manager-timeout', '60'],
            output='screen',
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            namespace=namespace,
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                kinematics_config,
                joint_limits_config,
                ompl_planning_pipeline_config,
                trajectory_execution,
                moveit_controllers,
                planning_scene_monitor_parameters,
            ],
        ),
    ]

    if use_rviz:
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
                parameters=[
                    {'robot_description': robot_description},
                    {'robot_description_semantic': robot_description_semantic},
                    kinematics_config,
                    joint_limits_config,
                    ompl_planning_pipeline_config,
                ],
            )
        )

    return nodes


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
                        FindPackageShare('pnp_dual_fr3_moveit_config'),
                        'config',
                        'pnp_dual_fr3_ros_controllers.yaml',
                    ]
                ),
                description='ROS 2 control controller configuration file.',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('franka_fr3_moveit_config'), 'rviz', 'moveit.rviz']
                ),
                description='RViz configuration file.',
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
