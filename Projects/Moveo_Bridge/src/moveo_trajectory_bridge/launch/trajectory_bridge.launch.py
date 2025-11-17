from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveo_trajectory_bridge',
            executable='trajectory_bridge',
            name='moveo_trajectory_bridge',
            output='screen',
            parameters=[{
                'trajectory_topic': '/joint_trajectory',
                'klipper_ws_url': 'ws://localhost:7125/websocket',
                'axis_map': ['A','B','C','D','E','U'],
                'position_unit': 'radians',  # or 'degrees'
                'buffer_lead_time': 0.5,      # seconds of future motion to maintain
                'min_flush_batch': 1,
                'max_flush_batch': 20,
                'flush_period': 0.05          # seconds
            }]
        )
    ])
