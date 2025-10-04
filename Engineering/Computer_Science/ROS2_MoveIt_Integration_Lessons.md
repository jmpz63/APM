# ROS 2 + MoveIt Integration Lessons Learned
*Extracted from moveo_bridge_ws Onboarding for APM Knowledge Base*

## 🎯 **Mission-Critical Integration Patterns**

### **ROS 2 → Klipper Pipeline Architecture**
```
MoveIt Plan (JointTrajectory) 
    ↓
FollowJointTrajectory Action Server
    ↓  
Trajectory Bridge (HTTP batching & clamping)
    ↓
Moonraker API → Klipper MANUAL_STEPPER
    ↓
Physical Hardware Motion
```

**Key Success Pattern:** Explicit stage verification with logging before hardware actuation

## 🔧 **Critical Technical Lessons**

### **Environment Setup (High Priority)**
```bash
# ALWAYS source both - terminal failures traced to missing sourcing
source /opt/ros/humble/setup.bash
source ~/workspace/install/setup.bash

# Add to ~/.bashrc for persistence:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/moveo_bridge_ws/install/setup.bash" >> ~/.bashrc
```

### **MoveIt Planning Error Resolution**
**Error Code -2 (GetMotionPlan fails):**
- **Root Cause:** Incomplete start_state (partial joint specification)
- **Solution:** Always populate RobotState with ALL group joints
- **Validation:** Verify joint delta > 0.01 rad for meaningful motion

### **Trajectory Bridge Integration**
**Critical Requirements:**
- Publish to `/joint_trajectory` topic
- Minimum 2 trajectory points with positions + time_from_start
- Verify "Loaded trajectory" confirmation before assuming success

## 🏗️ **Proven System Architecture**

### **4-Terminal Headless Control Setup (WORKING)**
```bash
# Terminal 1: MoveIt Stack  
ros2 launch moveo_moveit_config demo.launch.py

# Terminal 2: Action Server
ros2 run fjt_adapter fjt_adapter_node

# Terminal 3: Trajectory Bridge  
ros2 run trajectory_bridge trajectory_bridge_node

# Terminal 4: Control Interface
ros2 action send_goal /moveo_controller/follow_joint_trajectory ...
```

## 📊 **Hardware Specifications Framework**

### **Joint Specification Template**
```yaml
joint_N:
  type: revolute
  motor_model: "17HS24-0644S"  
  gear_ratio: "direct_drive"
  position_limits: [-180, +180]  # degrees
  velocity_limit: 30  # deg/s
  acceleration_limit: 45  # deg/s²
  torque_rating: 0.64  # Nm
```

### **Safety Layer Hierarchy**
1. **Planning Level:** MoveIt velocity/acceleration limits
2. **Bridge Level:** Hard-coded min/max position clamping  
3. **Firmware Level:** Klipper position_min/max + current limits
4. **Hardware Level:** Stepper driver stall detection

## 🚨 **Common Failure Modes & Resolutions**

| Issue | Root Cause | Resolution | Severity |
|-------|------------|------------|----------|
| `ros2: command not found` | Missing environment sourcing | Source setup.bash files | Medium |
| RViz MotionPlanning panel missing | Plugin path stripped by env -i | Avoid env clearing; verify packages | Low |
| No physical motion after plan | Start == Goal (zero delta) | Increase target angle > 0.01 rad | Medium |
| Trajectory not accepted | Wrong topic/empty points | Use /joint_trajectory with ≥2 points | High |

## 🔄 **Integration Best Practices**

### **Documentation Centralization**
- **Pattern:** Single source of truth in Onboarding folder
- **Anti-Pattern:** Duplicated hardware specs across packages
- **Solution:** Reference centralized docs, avoid editing copies

### **Progressive Validation Strategy**
1. **Single-Joint Baseline** (✅ Achieved)
2. **Multi-Joint Coordination** (Next phase)  
3. **Closed-Loop Feedback** (Future enhancement)
4. **Safety Layer Integration** (Continuous)

### **Debugging Protocol**
```bash
# 1. Verify environment
ros2 node list | grep moveit
ros2 topic list | grep trajectory

# 2. Check message flow  
ros2 topic echo /joint_trajectory --once

# 3. Validate hardware connection
curl http://localhost:7125/printer/info

# 4. Monitor execution logs
ros2 service call /moveo_controller/query_state ...
```

## 📈 **Performance Metrics**

### **Baseline Achievement (2025-10-02)**
- ✅ **Single-Joint Control:** Complete MoveIt → Hardware pipeline
- ✅ **Planning Reliability:** Error-free trajectory generation  
- ✅ **Hardware Actuation:** Verified physical motion execution
- ✅ **Integration Stability:** 4-terminal setup reproducible in ≤30 minutes

### **Success Criteria Framework**
- **Functional:** Plan joint to target; observe physical reproduction  
- **Reproducibility:** Documented steps achieve same state consistently
- **Onboarding Speed:** New AI agent productive within 30 minutes
- **Documentation Currency:** Onboarding artifacts updated with system evolution

## 🔮 **Future Architecture Extensions**

### **Planned Enhancements**
- Multi-joint synchronized execution with velocity scaling
- Real-time feedback loop: Klipper → ROS 2 /joint_states  
- Simulation environment parity (Gazebo physics integration)
- Emergency stop and soft-limit monitoring systems
- Automated regression testing with dry-run validation

### **Scalability Considerations** 
- Payload-aware trajectory heuristics
- Thermal management and current monitoring
- Network latency compensation for real-time control
- Modular joint configuration for different manipulator variants

---

## 🎓 **Key Learning: Integration vs Implementation**

**Critical Insight:** Success came from focusing on **integration patterns** rather than individual component implementation. The winning approach was:

1. **Explicit Pipeline Stages** with clear handoff protocols
2. **Comprehensive Environment Management** (sourcing, dependencies)  
3. **Progressive Validation Strategy** (single-joint → multi-joint)
4. **Centralized Documentation** with living status updates

**Pattern Recognition:** Complex robotics systems succeed through **architecture discipline** more than algorithmic sophistication.

---

*Source: moveo_bridge_ws Onboarding (2025-10-02)*  
*Integration Status: Single-joint baseline achieved; multi-joint coordination next*  
*Knowledge Transfer: Complete pipeline documentation with 30-minute reproduction target*