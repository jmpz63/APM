#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    dry_run = LaunchConfiguration('dry_run')
    manual_stepper = LaunchConfiguration('manual_stepper')

    moveit_pkg = get_package_share_directory('moveo_moveit_config')
    demo_launch = os.path.join(moveit_pkg, 'launch', 'demo.launch.py')

    # Reuse existing MoveIt demo launch (it already spawns robot_state_publisher + move_group + optional RViz)
    moveit_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch),
        launch_arguments={'use_rviz': use_rviz}.items()
    )

    # Shadow joint_states publisher (mirrors last JointTrajectory point)
    shim_node = Node(
        package='moveo_trajectory_bridge',
        executable='joint_state_shim',
        name='joint_state_shim',
        output='screen',
        parameters=[{'trajectory_topic': '/joint_trajectory', 'publish_rate': 20.0}]
    )

    # Relay from DisplayTrajectory -> JointTrajectory (if using RViz MotionPlanning panel)
    relay_node = Node(
        package='moveo_trajectory_bridge',
        executable='moveit_display_relay',
        name='moveit_display_relay',
        output='screen'
    )

    # Trajectory bridge to hardware (manual stepper mode for joint1 only)
    bridge_params = [
        {'trajectory_topic': '/joint_trajectory'},
        {'dry_run': LaunchConfiguration('dry_run')},
        {'manual_stepper_mode': LaunchConfiguration('manual_stepper')},
        {'manual_stepper_name': 'joint1'},
        {'manual_stepper_joint_index': 0},
        {'use_http_transport': True},
        {'moonraker_base_url': 'http://localhost:7125'},
        # Conservative joint limits (rad) single-joint for now (matching ±1.0821 rad ~62°)
        {'joint_position_limits': [-1.0821, 1.0821]}
    ]
    bridge_node = Node(
        package='moveo_trajectory_bridge',
        executable='trajectory_bridge',
        name='trajectory_bridge',
        output='screen',
        parameters=bridge_params
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('manual_stepper', default_value='true'),
        moveit_include,
        shim_node,
        relay_node,
        bridge_node
    ])
