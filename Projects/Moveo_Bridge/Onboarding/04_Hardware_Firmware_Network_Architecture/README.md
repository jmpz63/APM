# Hardware / Firmware / Network Architecture

High-level overview of mechanical, electrical, and networking components in the Moveo arm system.

## Contents
- Mechanical: joints, actuators, transmission
- Firmware: Klipper + Moonraker + (future) feedback integration
- ROS 2 Interface: trajectory bridge, planning stack
- Network: Host (ROS 2) <-> Moonraker (HTTP/WebSocket) <-> MCU
- Safety & Limits: soft limits, current limits, thermal considerations
- Joint Datasheets & Specs (see below)

## Directory Structure
```
04_Hardware_Firmware_Network_Architecture/
  README.md (this file)
  HARDWARE_NOTES.md (canonical actuator + transmission spec; maintain THIS copy)
  joint_datasheets/ (add PDFs, images, raw manufacturer datasheets)
```

## Joint Spec Workflow
1. Populate table in `HARDWARE_NOTES.md`.
2. Attach manufacturer PDFs in `joint_datasheets/`.
3. Run (future) script to derive velocity/accel limits for MoveIt & runtime bridge.
4. Commit both updated spec + regenerated `joint_limits.yaml`.

## Data Flow (Simplified)
```
MoveIt Plan (JointTrajectory)
          |
          v
Trajectory Bridge (batching, clamping, HTTP send)
          |
          v
Moonraker -> Klipper (MANUAL_STEPPER commands)
          |
          v
Motors / Mechanics
```
(Planned feedback path will feed joint state estimates back into ROS 2.)

## Safety / Integrity Layers
- Planning limits (MoveIt) prevent unrealistic velocity requests.
- Bridge clamps to hard-coded joint min/max + (future) velocity cap.
- Klipper enforces `position_min/max` (soft) and driver current & stall detection (if enabled).
- Manual review of `HARDWARE_NOTES.md` before changing accelerations.

## Next Steps
- Fill in missing joint specs (J3–J6).
- Add measurement log (mass, CoM, link lengths) for torque model.
- Script generation of dynamic limits.
- Implement joint state feedback source.

## Overview
A high-level view of components enabling simulation-to-physical execution.

```
        +------------------+            +------------------+
        |   MoveIt 2       |  plans     |  DisplayTrajectory|
        | (move_group)     +----------->+  Relay (optional) |
        +---------+--------+            +---------+--------+
                  |  JointTrajectory               |
                  v                                v
        +--------------------------+    +---------------------------+
        |  Trajectory Bridge       |    | Future: Feedback Node     |
        | (JointTrajectory -> G-code)   | (Moonraker -> /joint_states)|
        +---------------+----------+    +---------------+-----------+
                        | HTTP POST (Moonraker /printer/gcode/script)
                        v
                +------------------+   stepper events   +------------------+
                |   Moonraker      +------------------->+     Klipper       |
                | (HTTP API)       |                    | (Firmware)        |
                +---------+--------+                    +---------+---------+
                          | manual_stepper                         |
                          v                                        v
                     Physical Joint1 (others later)          Future joints
```

## Components
- **moveo_description**: URDF/Xacro (geometry, joints, visuals, collisions).
- **moveo_moveit_config**: SRDF, kinematics, joint limits, planning pipeline.
- **moveo_trajectory_bridge**: Streams JointTrajectory segments as G-code / MANUAL_STEPPER with limit enforcement.
- **Moonraker**: HTTP gateway to Klipper; provides /printer/gcode/script endpoint.
- **Klipper**: Firmware interpreting MANUAL_STEPPER for joint actuation.

## Network Assumptions
- Moonraker reachable at `http://192.168.50.11:7125` (adjust via parameter `moonraker_base_url`).
- No API key currently enforced (X-Api-Key header optional). Add to bridge params if enabled later.

## Manual Stepper Configuration (Summary)
- Klipper `printer.cfg` defines `manual_stepper` for joint1 with gear ratio and soft limits matching MoveIt.
- Movement units: currently degrees translated from radians at bridge level.

## Planned Additions
- Additional joints manual_stepper definitions.
- Feedback node polling Moonraker (e.g., querying position or using macros) -> publishes `/joint_states`.
- Safety layer (limit / e-stop monitoring) before sending G-code.

## Joint Datasheets
Place any per-joint mechanical / electrical spec PDFs or markdown summaries in `joint_datasheets/`.
