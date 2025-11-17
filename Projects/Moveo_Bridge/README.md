# BCN3D Moveo ROS 2 Bridge Project


📋 **AUTHORITATIVE SPECS**: All joint ranges, limits, and hardware parameters are defined in `printer.cfg`

🤖 **Complete ROS 2 integration for BCN3D Moveo 6-DOF robotic arm using Klipper firmware**

**Current Status**: 2/6 joints operational (Joint 1 + dual-motor Joint 2) with full TMC5160 control

## 🚀 Quick Start (4-Terminal Headless Control)

**Terminal 1: ROS 2 System**
```bash
cd ~/moveo_bridge_ws
source install/setup.bash
ros2 launch moveo_moveit_config demo.launch.py
```

**Terminal 2: Trajectory Bridge**
```bash
ros2 run moveo_trajectory_bridge trajectory_bridge
```

**Terminal 3: Test Movement**
```bash
python3 test_send_fjt_goal.py
```

**Terminal 4: Direct Klipper Control (Optional)**
```bash
# Enable joints
curl -X POST "http://localhost:7125/printer/gcode/script" -H "Content-Type: application/json" -d '{"script": "JOINT2_ENABLE"}'
# Move Joint 2
curl -X POST "http://localhost:7125/printer/gcode/script" -H "Content-Type: application/json" -d '{"script": "JOINT2_PLUS DISTANCE=15"}'
```

## 📁 Project Structure

### Hardware Configuration
- `printer.cfg` - **Complete Klipper configuration** (TMC5160 drivers, position limits, safety macros)

### ROS 2 Packages
- `src/moveo_description/` - Robot URDF, meshes, joint specifications
- `src/moveo_moveit_config/` - MoveIt 2 planning configuration  
- `src/moveo_trajectory_bridge/` - **HTTP bridge**: ROS ↔ Klipper communication

### Control Scripts
- `test_send_fjt_goal.py` - **Main working control** (FollowJointTrajectory action)
- `working_moveit_control.py` - MoveIt integration interface
- `precise_joint_control.py` - Individual joint control

### Documentation Hub  
- `Onboarding/01_Project_Manual.md` - Complete step-by-step guide
- `Onboarding/02_Progress_Log.md` - Development timeline & status
- `KNOWLEDGE_INDEX.md` - **Central knowledge management**

## ✅ Current Capabilities

### Joint 1 (Base Rotation) - See printer.cfg for authoritative specs
- **Range**: -90° to +81.5° (175.5° total)
- **Endstop**: PF0 pin, calibrated at 81.3°
- **TMC5160**: 0.6A current, StealthChop enabled
- **Status**: ✅ Fully operational

### Joint 2 (Shoulder - Dual Motor) - See printer.cfg for authoritative specs
- **Range**: -135° to +90° (225° total range)
- **Motors**: joint2 (Motor-8) + joint2b (Motor-7) **synchronized**
- **Endstop**: PF2 pin at 90° upper limit
- **TMC5160**: 0.6A + 0.6A currents (matched)
- **Gear Ratio**: 76:14 (calibrated)
- **Status**: ✅ Fully operational

### ⚠️ Critical Safety: Joint 2 Synchronized Movement
Joint 2 uses **dual mechanically-linked motors** that MUST move together:
```bash
# ✅ Correct - Use coordinated macros
JOINT2_PLUS DISTANCE=15 SPEED=10

# ✅ Correct - Manual with SYNC
MANUAL_STEPPER STEPPER=joint2 MOVE=15 SYNC=0
MANUAL_STEPPER STEPPER=joint2b MOVE=15 SYNC=1

# ❌ NEVER move independently - will damage mechanics!
```

## 🔧 Setup & Build

```bash
# Install dependencies
sudo apt install ros-humble-moveit ros-humble-joint-trajectory-controller

# Build workspace  
cd ~/moveo_bridge_ws
colcon build
source install/setup.bash

# Verify Klipper is running
systemctl status klipper
```

## 📊 Hardware Status

| Component | Status | Notes |
|-----------|--------|-------|
| BTT Octopus Max EZ | ✅ Working | 24V power stable |
| TMC5160 Motor-6 | ✅ Ready | Reserved for Joint 3 |
| TMC5160 Motor-7 | ✅ Working | Joint 2b @ 0.2A |
| TMC5160 Motor-8 | ✅ Working | Joint 2 @ 0.6A |
| TMC5160 Motor-5 | ❌ Hardware Fault | s2vsa=1 error |
| Joints 3-6 | 🔄 Ready | Hardware prepared |

## 🎯 Next Steps
1. **Joints 3-6 Integration**: Expand to full 6-DOF control
2. **Multi-Joint Trajectories**: Complex coordinated movements  
3. **Performance Tuning**: Speed and precision optimization
4. **Advanced Planning**: Collision avoidance and path planning

---
📖 **For detailed technical information**: See `KNOWLEDGE_INDEX.md`  
📝 **For step-by-step procedures**: See `Onboarding/01_Project_Manual.md`