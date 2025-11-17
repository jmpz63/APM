# Moveo Bridge Project - Knowledge Index

## Project Overview
**BCN3D Moveo 6-DOF Robotic Arm with ROS 2 Integration**
- **Hardware**: BTT Octopus Max EZ + TMC5160 drivers + 24V motors
- **Firmware**: Klipper/Moonraker for low-level motor control
- **Software**: ROS 2 Humble + MoveIt 2 for high-level planning
- **Communication**: HTTP bridge between ROS and Klipper
- **Status**: 2 joints operational (Joint 1 + Joint 2 dual-motor)

## 📚 LEARN IT - Knowledge Acquisition

### Hardware Discoveries
- **TMC5160 Driver Issues**: Motor-5 position has hardware fault, Motor-7/8 functional with reduced current
- **Dual-Motor Joint 2**: Requires synchronized movement, mechanically linked motors
- **Endstop Mapping**: M1-STOP=PF0, M2-STOP=PF2, M3-STOP=PF4, etc.
- **Power Requirements**: 24V supply stable, individual motor circuits isolated

### Software Learnings
- **ROS Integration**: FollowJointTrajectory action works reliably, MoveGroup needs more setup
- **Klipper Commands**: MANUAL_STEPPER with SYNC parameters for coordinated movement
- **TMC Configuration**: StealthChop + reduced current (0.2A-0.6A) overcomes hardware faults
- **Position Limits**: See printer.cfg (AUTHORITATIVE) - Joint1 [-90°, +81.5°], Joint2 [0°, 179.5°]

### Control Methods That Work
1. **Python Scripts**: Direct ROS action calls
2. **Command Line**: `ros2 action send_goal` with proper trajectory format
3. **Headless Control**: 4-terminal setup bypasses GUI complexity
4. **HTTP Bridge**: Moonraker API reliable for Klipper communication

## 📋 GOVERNANCE - Authoritative Sources

### 🤖 Configuration Authority
- **printer.cfg** - GOVERNING DOCUMENT for all joint specifications, ranges, limits
- **Hardware Parameters** - TMC settings, endstop configurations, gear ratios
- **Safety Limits** - position_min/max values are authoritative for all systems
- **All Documentation** - Must reference printer.cfg for accuracy and consistency

### 📖 Documentation Hierarchy
1. **printer.cfg** - Primary authority for hardware specifications
2. **README.md** - Project overview (references printer.cfg)
3. **KNOWLEDGE_INDEX.md** - This file - knowledge organization (references printer.cfg)
4. **Technical docs** - Implementation details (must align with printer.cfg)


## 📝 DOC IT - Documentation Structure

### Core Documentation
- **[Project Manual](Onboarding/01_Project_Manual.md)**: Step-by-step implementation guide
- **[Progress Log](Onboarding/02_Progress_Log.md)**: Chronological development history
- **[Hardware Notes](Onboarding/04_Hardware_Firmware_Network_Architecture/)**: Detailed hardware specifications
- **[Tips & Tricks](Onboarding/05_Tips_Tricks.md)**: Operational knowledge and shortcuts

### Configuration Files
- **[printer.cfg](printer.cfg)**: Complete Klipper configuration with TMC drivers
- **[ROS Launch Files](src/)**: MoveIt and trajectory bridge configurations
- **[URDF Models](src/moveo_description/)**: Robot description and visual models

### Safety Documentation
- **Joint 2 Synchronized Movement**: Critical requirement for dual-motor coordination
- **TMC Driver Limitations**: Hardware fault mitigation strategies
- **Position Limits**: Software safeguards preventing mechanical damage

## 🗂️ INDEX IT - Knowledge Organization

### By Component
```
Hardware/
├── BTT_Octopus_Max_EZ/
│   ├── TMC5160_Drivers/
│   ├── Endstop_Mapping/
│   └── Power_Distribution/
├── BCN3D_Moveo_Arm/
│   ├── Joint_Specifications/
│   ├── Mechanical_Limits/
│   └── Motor_Assignments/
└── Sensors_Endstops/

Software/
├── Klipper_Firmware/
│   ├── Motor_Configuration/
│   ├── TMC_Settings/
│   └── Safety_Macros/
├── ROS2_Integration/
│   ├── MoveIt_Config/
│   ├── Trajectory_Bridge/
│   └── Action_Servers/
└── Control_Interfaces/

Procedures/
├── Calibration/
├── Troubleshooting/
├── Testing/
└── Maintenance/
```

### By Development Phase
1. **Phase 1 Complete**: Single joint (Joint 1) with full ROS integration
2. **Phase 2 Complete**: Dual-motor Joint 2 with TMC control
3. **Phase 3 Pending**: Multi-joint coordination (Joints 3-6)
4. **Phase 4 Pending**: Advanced trajectory planning and execution

### By Priority Level
- **🔴 Critical**: Safety requirements, synchronized movement, position limits
- **🟡 Important**: TMC configurations, calibration procedures, ROS integration
- **🟢 Nice-to-have**: Advanced features, optimization, monitoring

## 🚀 PUSH IT - Knowledge Sharing & Implementation

### Git Repository Structure
```
moveo_bridge_ws/
├── README.md                    # Project overview and quick start
├── KNOWLEDGE_INDEX.md           # This file - central knowledge hub
├── Onboarding/                  # Comprehensive documentation
├── src/                         # ROS 2 source packages
├── printer.cfg                  # Klipper configuration
└── test_send_fjt_goal.py       # Working control examples
```

### Implementation Checklist
- [x] **Hardware Setup**: BTT Octopus + TMC drivers configured
- [x] **Joint 1 Operational**: Full range with endstop protection
- [x] **Joint 2 Dual-Motor**: Synchronized movement with TMC control
- [x] **ROS Integration**: Working trajectory execution
- [x] **Safety Systems**: Position limits and synchronized movement
- [ ] **Joints 3-6 Integration**: Expand to full 6-DOF operation
- [ ] **Advanced Planning**: Complex multi-joint trajectories
- [ ] **Performance Optimization**: Speed and precision tuning

### Knowledge Transfer Protocol
1. **Document as you discover**: Real-time progress logging
2. **Test before committing**: Validate all configurations
3. **Share working examples**: Maintain test_send_fjt_goal.py
4. **Update indexes**: Keep this knowledge map current
5. **Version control**: Git commits with descriptive messages

### Current Status Summary
**October 6, 2025**: 
- 2/6 joints operational with full TMC control
- ROS 2 trajectory execution working
- Safety systems implemented and documented
- Ready for expansion to remaining joints
- All critical knowledge captured and indexed

## 🔄 Continuous Learning Loop
1. **Experiment** → Document findings in Progress Log
2. **Implement** → Update configurations and code
3. **Validate** → Test thoroughly and record results  
4. **Document** → Update manuals and knowledge base
5. **Share** → Commit to repository with clear messages

---
*This index serves as the central hub for all project knowledge. Keep it updated as the project evolves.*