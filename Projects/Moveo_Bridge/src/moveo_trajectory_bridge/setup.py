from setuptools import find_packages, setup

setup(
    name='moveo_trajectory_bridge',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/moveo_trajectory_bridge']),
        ('share/moveo_trajectory_bridge', ['package.xml']),
        ('share/moveo_trajectory_bridge/launch', [
            'launch/trajectory_bridge.launch.py',
            'launch/joint1_manual_stepper_demo.launch.py',
            'launch/moveit_display_relay.launch.py',
            'launch/joint1_bridge_only.launch.py',
            'launch/joint1_with_feedback.launch.py',
            'launch/fjt_adapter.launch.py'
        ]),
        ('share/moveo_trajectory_bridge', ['README.md'])
    ],
    install_requires=[
        'setuptools',
        'websockets>=11.0.0',
    ],
    zip_safe=True,
    maintainer='arm1',
    maintainer_email='jmpz63.jp@gmail.com',
    description='MoveIt JointTrajectory to Klipper G-code streaming bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_bridge = moveo_trajectory_bridge.trajectory_bridge:main',
            'dummy_publisher = moveo_trajectory_bridge.dummy_publisher:main',
            'joint_state_shim = moveo_trajectory_bridge.joint_state_shim:main',
            'joint1_test_publisher = moveo_trajectory_bridge.joint1_test_publisher:main',
            'joint1_ratio_check = moveo_trajectory_bridge.joint1_ratio_check:main'
            , 'moveit_display_relay = moveo_trajectory_bridge.moveit_display_relay:main'
            , 'fjt_adapter = moveo_trajectory_bridge.fjt_adapter:main'
        ],
    },
)
