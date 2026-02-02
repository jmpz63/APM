from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    relay_params = {
        'display_topic': '/move_group/display_planned_path',
        'output_topic': '/joint_trajectory',
        'only_first': True,
        # Provide joints_subset as string array (placeholder values removed means autodetect all joints)
            'joints_subset': ['joint1','joint2','joint3','joint4','joint5'],
        'position_unit': 'radians'
    }
    return LaunchDescription([
        Node(
            package='moveo_trajectory_bridge',
            executable='moveit_display_relay',
            name='moveit_display_relay',
            output='screen',
            parameters=[relay_params]
        )
    ])