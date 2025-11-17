#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveo_trajectory_bridge',
            executable='fjt_adapter',
            name='fjt_adapter',
            output='screen',
            parameters=[
                {'publish_topic': '/joint_trajectory'},
                {'controller_name': 'manipulator_controller'},
                {'action_ns': 'follow_joint_trajectory'},
                {'wait_for_execution': True},  # Set to False for old optimistic behavior
                {'feedback_rate_hz': 2.0}     # Feedback frequency during execution
            ]
        ),
        Node(
            package='moveo_trajectory_bridge',
            executable='trajectory_bridge',
            name='trajectory_bridge',
            output='screen',
            parameters=[
                {'trajectory_topic': '/joint_trajectory'},
                {'dry_run': False},
                {'manual_stepper_mode': True},
                {'manual_stepper_joint_index': 0},
                {'manual_stepper_name': 'joint1'},
                {'use_http_transport': True},
                {'moonraker_base_url': 'http://localhost:7125'},
                {'joint_position_limits': [-1.0821, 1.0821, -1.5708, 1.5708, -2.0, 2.0, -2.0, 2.0, -3.14159, 3.14159, -2.0, 2.0]},
            ]
        )
    ])
