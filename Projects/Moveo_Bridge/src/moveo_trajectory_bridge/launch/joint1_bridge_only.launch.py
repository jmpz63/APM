from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_params = {
        'trajectory_topic': '/joint_trajectory',
        'klipper_ws_url': 'ws://localhost:7125/websocket',
        'axis_map': ['A','B','C','D','E','U'],
        'position_unit': 'radians',
        'min_flush_batch': 1,
        'max_flush_batch': 20,
        'flush_period': 0.05,
        'dry_run': False,
        'manual_stepper_mode': True,
        'manual_stepper_joint_index': 0,
        'manual_stepper_name': 'joint1',
        'manual_stepper_units': 'deg',
        'manual_stepper_default_speed': 30.0,
        'manual_stepper_speed_cap': 45.0,
        'joint_position_limits': [
            -1.0821, 1.0821,  # joint1
            -3.14, 3.14,      # joint2
            -3.14, 3.14,      # joint3
            -3.14, 3.14,      # joint4
            -3.14, 3.14,      # joint5
            -3.14, 3.14       # joint6
        ],
        'limit_mode': 'clamp',
        'use_http_transport': True,
        'moonraker_base_url': 'http://localhost:7125'
    }

    return LaunchDescription([
        Node(
            package='moveo_trajectory_bridge',
            executable='trajectory_bridge',
            name='trajectory_bridge_joint1',
            output='screen',
            parameters=[bridge_params]
        )
    ])
