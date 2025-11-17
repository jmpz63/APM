# moveo_moveit_config

MoveIt configuration scaffold for the Moveo arm.

## Dual Motor Joint (Joint2)
Joint2 is physically driven by two TMC5160 RGB drivers (motors 7 & 8 on the BTT Octo Max EZ). In the robot kinematic model we expose only a single revolute `joint2`. The firmware (Klipper) should mirror/lockstep the paired drivers so MoveIt and ROS see a single DOF.

No logical `joint2b` remains in the URDF or SRDF. If later you require per‑motor current or diagnostics, expose them as separate sensor/status topics—not additional kinematic joints.

## Package Contents
- `urdf/moveo.urdf.xacro` (includes upstream description)
- `config/` kinematics, OMPL planners, joint limits, SRDF template, allowed collisions
- `launch/demo.launch.py` basic bringup (robot_state_publisher + move_group + optional RViz)

## Next Steps (Manual)
1. Refine URDF with accurate inertias, masses, visuals.
2. Generate a full SRDF (MoveIt Setup Assistant) replacing `srdf_template.srdf`.
3. Add end effector definition if gripper/tooling exists.
4. Tune planner limits (velocities/accelerations) to match hardware.
5. Integrate trajectory execution pipeline (bridge to Klipper) once planning validated.

## Usage
```bash
source /opt/ros/humble/setup.bash
cd ~/moveo_bridge_ws
colcon build --merge-install --packages-select moveo_description moveo_moveit_config
source install/setup.bash
ros2 launch moveo_moveit_config demo.launch.py
```

## Notes
- Xacro is processed in Python; ensure `python3-xacro` (ROS Humble `ros-humble-xacro`) is installed.
- If MoveIt packages are missing, install via `sudo apt install ros-humble-moveit`.
