import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)
def generate_robot_nodes(context):
    robot_types = LaunchConfiguration('robot_types').perform(context)
    robot_ips = LaunchConfiguration('robot_ips').perform(context)
    arm_prefixes = LaunchConfiguration('arm_prefixes').perform(context)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    load_gripper = LaunchConfiguration('load_gripper').perform(context)
    joint_state_rate = int(LaunchConfiguration('joint_state_rate').perform(context))
    thread_priority = LaunchConfiguration('thread_priority').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'

    pnp_urdf_xacro_file = PathJoinSubstitution(
        [
            FindPackageShare('pnp_dual_fr3_description'),
            'robots',
            'pnp_dual_fr3',
            'pnp_dual_fr3.urdf.xacro',
        ]
    ).perform(context)

    robot_description = {
        'robot_description': xacro.process_file(
            pnp_urdf_xacro_file,
            mappings={
                'robot_types': robot_types,
                'robot_ips': robot_ips,
                'arm_prefixes': arm_prefixes,
                'hand': load_gripper,
                'use_fake_hardware': use_fake_hardware,
                'fake_sensor_commands': fake_sensor_commands,
                'ros2_control': 'true',
                'is_async': 'true',
                'thread_priority': thread_priority,
            },
        ).toprettyxml(indent='  ')
    }

    pnp_semantic_xacro_file = PathJoinSubstitution(
        [
            FindPackageShare('pnp_dual_fr3_description'),
            'robots',
            'pnp_dual_fr3',
            'pnp_dual_fr3.srdf.xacro',
        ]
    ).perform(context)

    robot_description_semantic = {
        'robot_description_semantic': xacro.process_file(
            pnp_semantic_xacro_file,
            mappings={
                'robot_types': robot_types,
                'arm_prefixes': arm_prefixes,
                'hand': load_gripper,
            },
        ).toprettyxml(indent='  ')
    }

    kinematics_config = {
        'robot_description_kinematics': load_yaml(
            'pnp_dual_fr3_moveit_config', 'config/kinematics.yaml'
        )
    }

    joint_limits_config = {
        'robot_description_planning': load_yaml(
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
    ompl_planning_yaml = load_yaml(
        'pnp_dual_fr3_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'pnp_dual_fr3_moveit_config', 'config/pnp_dual_fr3_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
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
        # 'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'franka_fr3_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d',
            rviz_full_config
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description, {'publish_robot_description': True}],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('pnp_dual_fr3_moveit_config'),
        'config',
        'pnp_dual_fr3_ros_controllers.yaml',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={'stdout': 'screen', 'stderr': 'screen'},
        on_exit=Shutdown(),
    )

    load_controllers = []
    for controller in ['left_arm_controller', 'right_arm_controller', 'joint_state_broadcaster']:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', controller,
                    '--controller-manager-timeout', '60',
                    '--controller-manager', PathJoinSubstitution([namespace, 'controller_manager']),
                ],
                output='screen',
            )
        )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[
            {'source_list': ['franka/joint_states'], 'rate': joint_state_rate}
        ],
    )

    nodes = [
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_publisher,
    ] + load_controllers

    if use_rviz:
        nodes.insert(0, rviz_node)

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_types',
                default_value="['fr3v2','fr3v2']",
                description='Types of the robot arms as a string list.',
            ),
            DeclareLaunchArgument(
                'robot_ips',
                default_value="['172.16.16.11','172.16.16.12']",
                description='IP addresses of the robot arms as a string list.',
            ),
            DeclareLaunchArgument(
                'arm_prefixes',
                default_value="['left','right']",
                description='Arm prefixes as a string list.',
            ),
            DeclareLaunchArgument(
                'load_gripper',
                default_value='false',
                description='Whether to load grippers.',
            ),
            DeclareLaunchArgument(
                'use_fake_hardware',
                default_value='true',
                description='Use fake hardware.',
            ),
            DeclareLaunchArgument(
                'fake_sensor_commands',
                default_value='true',
                description='Use fake sensor commands.',
            ),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Namespace for the robot.',
            ),
            DeclareLaunchArgument(
                'joint_state_rate',
                default_value='1000',
                description='Joint state publisher rate in Hz.',
            ),
            DeclareLaunchArgument(
                'thread_priority',
                default_value='50',
                description='Thread priority for the hardware interface.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Launch RViz.',
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
