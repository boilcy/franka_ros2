import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
    namespace = LaunchConfiguration('namespace').perform(context)
    load_gripper = LaunchConfiguration('load_gripper').perform(context)

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
                'ros2_control': 'false',
                'robot_types': robot_types,
                'robot_ips': robot_ips,
                'arm_prefixes': arm_prefixes,
                'hand': load_gripper,
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
    ompl_planning_pipeline_config['move_group'].update(
        load_yaml('pnp_dual_fr3_moveit_config', 'config/ompl_planning.yaml')
    )

    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml(
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

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
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

    return [run_move_group_node]


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
                'namespace',
                default_value='',
                description='Namespace for the robot.',
            ),
            DeclareLaunchArgument(
                'load_gripper',
                default_value='false',
                description='Whether to load grippers.',
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
