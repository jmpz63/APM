#!/usr/bin/env python3
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('moveo_moveit_config')
    desc_share = get_package_share_directory('moveo_description')

    use_rviz = LaunchConfiguration('use_rviz')

    xacro_path = os.path.join(desc_share, 'urdf', 'moveo.urdf.xacro')
    doc = xacro.process_file(xacro_path)
    robot_description = {'robot_description': doc.toxml()}

    srdf_path = os.path.join(pkg_share, 'config', 'srdf_template.srdf')
    with open(srdf_path, 'r') as f:
        srdf_content = f.read()
    robot_description_semantic = {'robot_description_semantic': srdf_content}

    kinematics_yaml_path = os.path.join(pkg_share, 'config', 'kinematics.yaml')
    ompl_yaml_path = os.path.join(pkg_share, 'config', 'ompl_planning.yaml')
    joint_limits_yaml_path = os.path.join(pkg_share, 'config', 'joint_limits.yaml')
    controllers_yaml_path = os.path.join(pkg_share, 'config', 'controllers.yaml')

    def load_yaml(path):
        try:
            with open(path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            return {}

    kinematics_yaml = load_yaml(kinematics_yaml_path)
    ompl_yaml = load_yaml(ompl_yaml_path)
    joint_limits_yaml = load_yaml(joint_limits_yaml_path)
    controllers_yaml = load_yaml(controllers_yaml_path)

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                {'robot_description_kinematics': kinematics_yaml},
                {'robot_description_planning': joint_limits_yaml.get('joint_limits', joint_limits_yaml)},
                {'planning_plugin': 'ompl_interface/OMPLPlanner'},
                controllers_yaml,
                # Disable octomap monitor to silence missing 3D sensor plugin error
                {'planning_scene_monitor': {
                    'publish_geometry_updates': True,
                    'publish_state_updates': True,
                    'publish_transforms_updates': True,
                    'octomap_monitor': False
                }},
            ]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'moveit.rviz')],
            parameters=[robot_description, robot_description_semantic]
        )
    ]

    return LaunchDescription([
    DeclareLaunchArgument('use_rviz', default_value='true'),
        *nodes
    ])
