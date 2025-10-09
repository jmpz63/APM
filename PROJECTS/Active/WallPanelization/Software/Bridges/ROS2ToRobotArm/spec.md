# Bridge: ROS 2 → Robot Arm Controller [PLANNED]

Purpose: Convert work instructions into robot motions and IO.

Inputs
- WorkInstruction msgs

Outputs
- Driver calls (vendor or ROS-I), tool IO commands

SLA
- Safe speeds, accel limits, soft limits, collision zones

Implementation
- ROS-Industrial or vendor-specific driver; simulate in Gazebo if available
