# APM Arm → ROS2 MoveIt Integration Guide

This guide documents how to bridge CAD/BIM to a MoveIt-ready ROS2 description for the APM robotic arm.

Key assumptions
- Arm is 5 DOF (joints 1–5), joint6 is spare (unused).
- Joint7 is a linear axis (X ~ 4 ft) acting as a base translation.
- Homing complete via Klipper; positions are consistent and zeroed.

## Overview
- CAD/BIM → URDF (robot_description)
- SRDF → semantic configuration
- Controllers → `ros2_control` interfaces
- MoveIt → Planning scene, kinematics, planning pipelines

## Data contract
- URDF links/joints: base → joint1 … joint5; linear axis joint7 along X.
- Joint types:
  - joint1..5: revolute, effort/position interfaces
  - joint7: prismatic (X), effort/position interface
- Limits and safety: velocity/acceleration consistent with hardware

## Steps
1) Export CAD to URDF
   - Use onshape-to-urdf or Fusion360 exporter; ensure correct origins and joint axes.
   - Define joint7 as prismatic (X). Leave joint6 as fixed or omitted.
2) Build the MoveIt config
   - Use `moveit2_setup_assistant` with your URDF.
   - Define planning groups: `arm_group` (joints1-5), `gantry_x` (joint7), optionally `arm_on_gantry` combining both.
   - Generate SRDF and kinematics.yaml.
3) Controllers
   - For simulation: fake joint driver or `ros2_control` PositionJointInterface.
   - For hardware: bridge MoveIt trajectory to your Klipper macro commands via a custom ROS2 node.
4) Test
   - Launch RViz with MoveIt.
   - Plan simple poses including linear X translation.

## Next
- Create a dedicated project hub under `Projects/Active/` with a minimal ROS2 workspace scaffold and MoveIt config.
- Implement the hardware bridge node mapping MoveIt trajectories to Klipper macros.
