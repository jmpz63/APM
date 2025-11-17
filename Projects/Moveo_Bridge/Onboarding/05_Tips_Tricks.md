# Tips & Tricks (Consolidated)

This file consolidates all quick-reference material. Individual files under `tips/` can still exist for modular editing, but this is the single authoritative hub.

## Contents
1. [Quick Read-Only Web Access](#quick-read-only-web-access-to-workspace-great-for-tablet--ipad-viewing)
2. [Headless MoveIt Control](#headless-moveit-control-working-2025-10-02)
3. [ROS 2 Commands](#ros-2-commands)
4. [Ubuntu / System Commands](#ubuntu--system-commands)
5. [Klipper / Moonraker G-code & API](#klipper--moonraker-g-code--api)
6. [Security Reminders](#security-reminders)
7. [Future Additions](#future-additions)

---

## Quick Read-Only Web Access to Workspace (Great for Tablet / iPad Viewing)
Serve the entire workspace (including `Onboarding/` docs) over HTTP on your LAN:
```bash
cd ~/moveo_bridge_ws
python3 -m http.server 9000 --bind 0.0.0.0
```
Then on another device (same network) open:
```
http://<BEELINK_LAN_IP>:9000/Onboarding/
```
Find IP quickly:
```bash
ip -4 addr show | grep -oP 'inet \K[0-9.]+'
```
Optional nicer Markdown rendering:
```bash
pip install grip
grip Onboarding/01_Project_Manual.md 9001 --username '' --password ''
```
Browse: `http://<BEELINK_LAN_IP>:9001/`

---

## Headless MoveIt Control (WORKING 2025-10-02) ⭐
**Status**: ✅ FULLY WORKING - Complete single-joint control via multiple interfaces

### 4-Terminal Setup (Required Every Time)

#### Terminal 1: MoveIt Stack
```bash
cd /home/arm1/moveo_bridge_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch moveo_moveit_config demo.launch.py
```
**Expected**: MoveIt starts, RViz crashes (ignore), "You can start planning now!"

#### Terminal 2: Execution Adapter  
```bash
cd /home/arm1/moveo_bridge_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run moveo_trajectory_bridge fjt_adapter --ros-args -p wait_for_execution:=true -p feedback_rate_hz:=2.0
```
**Expected**: "FollowJointTrajectory adapter active. Action server: /manipulator_controller/follow_joint_trajectory"

#### Terminal 3: Hardware Bridge
```bash
cd /home/arm1/moveo_bridge_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run moveo_trajectory_bridge trajectory_bridge --ros-args -p trajectory_topic:=/joint_trajectory -p dry_run:=false -p manual_stepper_mode:=true -p manual_stepper_joint_index:=0 -p manual_stepper_name:=joint1 -p use_http_transport:=true -p moonraker_base_url:=http://localhost:7125
```
**Expected**: "Subscribed to trajectory topic: /joint_trajectory"

#### Terminal 4: Control Commands
```bash
cd /home/arm1/moveo_bridge_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Method A: Python Scripts (Recommended)
python3 test_execution_feedback.py      # Proven working ✅
python3 working_moveit_control.py       # Interactive
python3 precise_joint_control.py        # Fine control

# Method B: Direct Commands (copy-paste below)
```

### Basic Single-Point Movements (Copy-Paste)
```bash
# Small 10° movement:
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.175], time_from_start: {sec: 2, nanosec: 0}}]}}" --feedback

# Medium 20° movement:
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.349], time_from_start: {sec: 3, nanosec: 0}}]}}" --feedback

# Return to center:
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.0], time_from_start: {sec: 2, nanosec: 0}}]}}" --feedback

# Large 30° movement (slow):
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.524], time_from_start: {sec: 4, nanosec: 0}}]}}" --feedback

# Negative direction -15°:
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [-0.262], time_from_start: {sec: 2, nanosec: 0}}]}}" --feedback

# Quick 5° movement:
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.087], time_from_start: {sec: 1, nanosec: 0}}]}}" --feedback
```

### Advanced Two-Point Trajectories (Smoother Motion)
```bash
# 5° movement (2-point):
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.044], time_from_start: {sec: 1, nanosec: 0}}, {positions: [0.087], time_from_start: {sec: 2, nanosec: 0}}]}}" --feedback

# 15° movement (2-point):
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.131], time_from_start: {sec: 1, nanosec: 500000000}}, {positions: [0.262], time_from_start: {sec: 3, nanosec: 0}}]}}" --feedback

# 30° movement (2-point):
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.262], time_from_start: {sec: 2, nanosec: 0}}, {positions: [0.524], time_from_start: {sec: 4, nanosec: 0}}]}}" --feedback
```

### Angle Reference & Troubleshooting

#### Quick Angle Conversion
| Degrees | Radians | Notes |
|---------|---------|-------|
| 5°      | 0.087   | Small test movement |
| 10°     | 0.175   | Safe test range |
| 15°     | 0.262   | Medium movement |
| 20°     | 0.349   | **TESTED WORKING** ✅ |
| 30°     | 0.524   | Large movement |
| 45°     | 0.785   | Medium-large movement |
| 60°     | 1.047   | Large movement |
| 75°     | 1.309   | Near maximum safe range |
| 81.5°   | 1.422   | **MAXIMUM RANGE** ✅ |
| -20°    | -0.349  | Negative direction |

#### Common Issues & Solutions
**Problem**: No arm movement
- ✅ Check all 4 terminals are running
- ✅ Verify Klipper/Moonraker connection at http://localhost:7125
- ✅ Ensure trajectory_bridge shows "Subscribed to trajectory topic"

**Problem**: Action rejected  
- ✅ Check joint angle is within limits (-90° to +81.5°)
- ✅ Verify time_from_start > 0

**Problem**: Movement too fast/slow
- ✅ Adjust `time_from_start: {sec: X}` value (higher = slower)

#### Success Indicators
- Terminal 2: "Goal accepted", "Execution feedback", "✅ SUCCESS"
- Terminal 3: HTTP requests to Moonraker visible
- **Physical arm moves to target position** 🎯

### System Status & Next Steps
**🎯 MILESTONE ACHIEVED (2025-10-02)**
- Python scripts: ✅ Working
- ROS2 commands: ✅ Working (single & 2-point trajectories)
- Hardware motion: ✅ Confirmed physical movement
- Execution feedback: ✅ Real-time progress updates
- 4-terminal setup: ✅ Reliable and reproducible

**Ready for Phase 2**: Multi-Joint Coordination (Joints 2-6) - See Project Manual for roadmap

### Status Check
```bash
# Verify all nodes running:
ros2 node list | grep -E "fjt_adapter|moveo_trajectory_bridge|move_group"

# Check action server:
ros2 action info /manipulator_controller/follow_joint_trajectory
```

**See**: `HEADLESS_CONTROL_REFERENCE.md` for complete guide

---

## ROS 2 Commands
### Environment Sourcing
```bash
source /opt/ros/humble/setup.bash
source ~/moveo_bridge_ws/install/setup.bash
```
Persist:
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/moveo_bridge_ws/install/setup.bash' >> ~/.bashrc
```
### Introspection
```bash
ros2 topic list
ros2 node list
ros2 param list
```
### Topic Echo & Publish
```bash
ros2 topic echo /joint_trajectory
ros2 topic pub --once /joint_trajectory trajectory_msgs/JointTrajectory "{joint_names:['joint1'], points:[{positions:[0.3], time_from_start:{sec:2}}]}"
```
### Services
```bash
ros2 service type /plan_kinematic_path
ros2 service call /plan_kinematic_path moveit_msgs/srv/GetMotionPlan "{motion_plan_request:{group_name:'manipulator'}}"
```
### Package / Executable Discovery
```bash
ros2 pkg list | grep moveo
ros2 run moveo_trajectory_bridge trajectory_bridge --help
```
### Launch Example
```bash
ros2 launch moveo_moveit_config demo.launch.py use_rviz:=false
```
### Debug / Misc
```bash
ros2 topic list | grep -i moveit
ros2 topic info /joint_trajectory
```


## Ubuntu / System Commands
### System Update & Packages
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential git curl wget jq tree tmux
```
### Monitoring
```bash
top
htop
free -h
df -h
sudo iotop
```
### Networking
```bash
ip -4 addr show
ip route
ping -c 4 8.8.8.8
ss -tlnp
```

#### Network Troubleshooting - Klipper Connection Issues 🌐
**Problem**: Can't connect to Klipper web interface or API

**Check Current IP Address:**
```bash
nmcli connection show --active    # Show active network connections
ip addr show wlo1                 # Check WiFi interface IP
ip -4 addr show | grep -v 127     # Show all non-localhost IPs
```

**Common Issue**: DHCP IP address changes
- **Symptom**: `192.168.50.11` worked yesterday, now doesn't
- **Cause**: Router assigned new IP from DHCP pool
- **Check logs**: `sudo journalctl -u NetworkManager --since "2 days ago" | grep "192.168.50"`

**Solutions:**
1. **Find current IP and use it**:
   ```bash
   # Get current IP
   ip addr show wlo1 | grep "inet " | awk '{print $2}' | cut -d/ -f1
   # Test connection
   curl "http://[CURRENT_IP]:7125/printer/info"
   ```

2. **Set DHCP reservation** (Router admin panel):
   - MAC Address: Check with `ip addr show wlo1`
   - Reserve specific IP (e.g., 192.168.50.11)

3. **Use hostname instead** (if mDNS works):
   ```bash
   curl "http://ARM1.local:7125/printer/info"
   ```

**Verify Services Running:**
```bash
sudo ss -tlnp | grep -E ":7125|:80"  # Check Moonraker (7125) and nginx (80)
systemctl status klipper moonraker   # Check service status
```

**Example IP Change Timeline:**
```
Oct 08 12:41 - IP was 192.168.50.11
Oct 10 08:32 - IP changed to 192.168.50.12 (DHCP lease renewal)
```
### Services
```bash
systemctl --no-pager status klipper moonraker
sudo systemctl restart klipper
sudo journalctl -u klipper -n 50 --no-pager
```
### Disk & Files
```bash
lsblk
mount | grep -i nvme
sudo fdisk -l
find . -maxdepth 2 -type f -name '*.launch.py'
```
### Users & Permissions
```bash
id
whoami
sudo usermod -aG dialout $USER
```
### Firewall (Optional)
```bash
sudo apt install -y ufw
sudo ufw allow 22/tcp
sudo ufw enable
sudo ufw status
```
### Archiving / Transfer
```bash
tar czf onboarding_snapshot_$(date +%F).tar.gz Onboarding/
scp onboarding_snapshot_$(date +%F).tar.gz user@host:/tmp/
```
### System Info Snapshot
```bash
uname -a
lsb_release -a
lscpu
```

---

## Klipper / Moonraker G-code & API

### Critical Configuration Files 🔧
**Location**: Your workspace now includes copies of essential config files
- `printer.cfg` - **Complete Klipper configuration** (995 lines, all 6 joints configured)
- `joint_configurations.cfg` - **Joint-only extract** for quick reference

**Active Config**: `/home/arm1/printer_data/config/printer.cfg`
**Backup in workspace**: `/home/arm1/moveo_bridge_ws/printer.cfg`

### Joint Configuration Summary
Your Klipper is configured with **ALL 6 JOINTS**:
```ini
[manual_stepper joint1]  # Base - Motor-2 (PE4/PE5) + TMC5160 + Endstop
[manual_stepper joint2]  # Shoulder Main - Motor-8 (PA10/PA9)  
[manual_stepper joint2b] # Shoulder Secondary - Motor-7 (PD3/PD2)
[manual_stepper joint3]  # Elbow - Motor-6 (PG15/PB3)
[manual_stepper joint4]  # Wrist Pitch - Motor-5 (PB5/PB4)
[manual_stepper joint5]  # Wrist Roll - Motor-4 (PB8/PB9)
[manual_stepper joint6]  # Tool Head - Motor-3 (PE1/PE0)
```

**Important**: Only Joint1 is currently used by your ROS system. Joints 2-6 are **configured but not integrated** with trajectory_bridge yet.

### Direct G-code Console Commands
Access via Mainsail/Fluidd web interface (http://localhost:7125) or Moonraker API:
```gcode
# Individual joint control
MANUAL_STEPPER STEPPER=joint1 MOVE=10 SPEED=5
MANUAL_STEPPER STEPPER=joint1 SET_POSITION=0

# TMC Driver diagnostics and control
DUMP_TMC STEPPER=joint2           # Show TMC register values (hex format)
DUMP_TMC                          # Show all TMC drivers
SET_TMC_CURRENT STEPPER=joint2 CURRENT=2.8 HOLDCURRENT=2.0  # Runtime current adjustment

# Multi-joint macros (built-in)
ARM_HOME          # Enable and zero all joints
ARM_STATUS        # Status of all 6 joints  
ARM_DEMO          # Demo sequence
ARM_DISABLE       # Disable all motors

# Joint1 specific (with endstop)
HOME_JOINT_BASE          # Home using endstop
JOINT_BASE_PLUS DISTANCE=20    # Move +20°
JOINT_BASE_MINUS DISTANCE=15   # Move -15°

# Joint2 dual-motor shoulder control
JOINT2_ENABLE     # Enable both shoulder motors
JOINT2_PLUS DISTANCE=15 SPEED=10       # Move shoulder positive
JOINT2_MINUS DISTANCE=15 SPEED=10      # Move shoulder negative  
JOINT2_SAFE_MOVE DISTANCE=20           # Move with position limit checking
JOINT2_SAFETY_WARNING                  # Display safety reminders
```

#### TMC Current Verification 🔍
**Note**: `DUMP_TMC` shows register values in hex, not the configured currents.
- `cs_actual` register: Shows current scaling factor (should be ~31 for full current)
- To see configured values: Check your `printer.cfg` file
- To change current runtime: Use `SET_TMC_CURRENT` command
- **See**: `TMC5160_Knowledge_Base.md` for complete TMC register documentation

---

## Security Reminders
- Prefer LAN / VPN (Tailscale) over raw port forwarding.
- Do not expose `python -m http.server` publicly—no auth.
- Keep system packages updated weekly.
- Consider API key for Moonraker if network is shared.

## Future Additions
Planned sections: Git Workflow, Limits Generation Script Usage, Feedback Node Operations.

---
Maintenance: When adding a new tip create subsection here; only add separate file if very large.
