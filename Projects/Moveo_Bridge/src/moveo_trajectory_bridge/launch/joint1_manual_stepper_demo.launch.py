from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Joint1 limits in radians (+/-1.0821), others wide placeholders.
    joint_limits = [
        -1.0821, 1.0821,   # joint1
        -3.14, 3.14,       # joint2
        -3.14, 3.14,       # joint3
        -3.14, 3.14,       # joint4
        -3.14, 3.14,       # joint5
        -3.14, 3.14        # joint6
    ]

    bridge_params = {
        'trajectory_topic': '/joint_trajectory',
        'klipper_ws_url': 'ws://localhost:7125/websocket',
        'axis_map': ['A','B','C','D','E','U'],
        'position_unit': 'radians',
        'min_flush_batch': 1,
        'max_flush_batch': 20,
        'flush_period': 0.05,
    'dry_run': False,                     # live mode (set True if you want simulation)
        'manual_stepper_mode': True,
        'manual_stepper_joint_index': 0,      # joint1
        'manual_stepper_name': 'joint1',
        'manual_stepper_units': 'deg',        # convert rad deltas -> degrees
        'manual_stepper_default_speed': 30.0, # deg/s default
        'manual_stepper_speed_cap': 45.0,     # deg/s cap for safety
        'joint_position_limits': joint_limits,
        'limit_mode': 'clamp'
    }

    test_pub_params = {
        'topic': '/joint_trajectory',
        'use_radians': True,
        'segment_time': 1.0,
        'one_shot': True
    }

    return LaunchDescription([
        Node(
            package='moveo_trajectory_bridge',
            executable='trajectory_bridge',
            name='trajectory_bridge_joint1',
            output='screen',
            parameters=[bridge_params]
        ),
        Node(
            package='moveo_trajectory_bridge',
            executable='joint1_test_publisher',
            name='joint1_test_pub',
            output='screen',
            parameters=[test_pub_params]
        )
    ])
