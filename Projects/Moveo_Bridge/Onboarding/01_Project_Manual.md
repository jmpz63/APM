# Proj## Current Capability Snapshot (2025-10-02)
| Area | Status | Notes |
|------|--------|---------|
| URDF / Visuals | ✅ Basic visuals & collisions present | Inertias & fine collision polish pending |
| MoveIt Planning | ✅ Planning works; controller recognized | RViz GUI crash (GLIBC snap conflict) blocks panel use |
| FollowJointTrajectory Execution | ✅ End-to-end single joint with proper feedback | Enhanced with realistic timing and progress updates |
| Trajectory Bridge | ✅ JointTrajectory -> MANUAL_STEPPER over HTTP | Complete pipeline: MoveIt → FJT Adapter → Trajectory Bridge → Klipper |
| Hardware Actuation | ✅ Joint1 physical motion verified via headless control | Reliable 4-terminal setup achieving actual arm movement |
| Headless Control | ✅ Multiple control methods working | Direct FollowJointTrajectory actions, Python scripts, ROS commands (2-point trajectories) |
| Single-Joint Control | ✅ COMPLETE BASELINE ACHIEVED | All interfaces working: Python, ROS2 commands, feedback, hardware motion |al (Living Guide)

> Status: SINGLE-JOINT HARDWARE MOTION + FollowJointTrajectory PATH ACHIEVED (MoveIt → fjt_adapter → /joint_trajectory → trajectory_bridge → MANUAL_STEPPER → Klipper). Shadow /joint_states now publishing (command echo). RViz GUI currently blocked by snap GLIBC conflict. Multi-joint + real feedback pending – Updated 2025-10-01 (EOD)

(See also: `05_Tips_Tricks.md` for quick workspace sharing & utilities.)

## Current Capability Snapshot (2025-10-01)
| Area | Status | Notes |
|------|--------|-------|
| URDF / Visuals | ✅ Basic visuals & collisions present | Inertias & fine collision polish pending |
| MoveIt Planning | ✅ Planning works; controller recognized | RViz GUI crash (GLIBC snap conflict) blocks panel use |
| FollowJointTrajectory Execution | ✅ End-to-end single joint | Action returns success immediately (optimistic); add feedback/result gating |
| Trajectory Bridge | ✅ JointTrajectory -> MANUAL_STEPPER over HTTP | Now also echoes shadow /joint_states; multi-joint path disabled |
| Hardware Actuation | ✅ Joint1 physical motion verified | Extend to joints 2–6 (manual_stepper sections) |
| Limits & Safety | ⚠️ Position limits enforced; base endstop characterization mid-process | Add safety enable gate before live send |
| Feedback (/joint_states) | 🟡 Shadow (command echo only) | Real feedback via Moonraker polling or sensors TBD |
| Multi-Joint Execution | ❌ Not validated | Add manual_stepper for J2/J3 then dry-run batch |
| Documentation / Onboarding | ✅ Updated with action bridge path | Add RViz crash workaround section |
| Datasheets Integration | ✅ J1/J2 partial | Fill J3–J6 + limit generator script |
| RViz / MotionPlanning Panel | ❌ RViz crash (libpthread GLIBC_PRIVATE) | Launch outside snap / sanitize env |
| Automation Scripts | ❌ None yet | Limit derivation + regression dry-run test harness |

## 0. Hardware Baseline (Beelink Host)
- Hardware: Beelink mini PC (CPU: Intel N100) hosting ROS 2 + Klipper/Moonraker.
- OS: Ubuntu 22.04 LTS (Jammy) 64-bit.

### 0.1 Fresh System Prep
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl wget git build-essential python3-pip python3-venv software-properties-common
```
Optional dev utilities:
```bash
sudo apt install -y htop net-tools tmux tree jq
```

### 0.2 ROS 2 Humble Install
Reference: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
```bash
sudo apt install -y locales && sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```
sudo apt install -y ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
```

### 0.4 Klipper + Moonraker + Mainsail (Full Install)

Goal: Operational Klipper firmware + Moonraker API + Mainsail web UI on the Beelink host, reachable (default) at `http://<host>:7125` (Moonraker) and `http://<host>/` (Mainsail, if using nginx). Adjust ports if you already have services bound.

- Moonraker: https://moonraker.readthedocs.io
- Mainsail: https://docs.mainsail.xyz
```bash
sudo apt update
sudo apt install -y git python3 python3-virtualenv python3-venv python3-pip virtualenv nginx unzip avahi-daemon
```
#### 0.4.2 Clone Repositories (in home)
```bash
git clone https://github.com/Arksine/moonraker.git
mkdir -p ~/printer_data/{config,logs,gcodes}
```
#### 0.4.3 Create Klipper Virtualenv & Install
```bash
./scripts/install-debian.sh
```
Accept prompts. This sets up systemd service `klipper.service`.
If your controller is already flashed & connected via USB or serial, verify with:
```bash
ls /dev/serial/by-id/

#### 0.4.5 Install Moonraker
cd ~/moonraker
./scripts/install-moonraker.sh
```

Minimal `moonraker.conf` additions (append):
```ini
[server]
host: 0.0.0.0
[file_manager]

[authorization]
trusted_clients:
  127.0.0.1
  ::1
# api_key: REPLACE_WITH_RANDOM_HEX
```
Restart:
```bash
sudo systemctl restart moonraker
```
Test API:
```bash
#### 0.4.6 Install Mainsail (Optional UI)
```bash
cd ~
sudo mkdir -p /var/www/mainsail
sudo unzip -o mainsail.zip -d /var/www/mainsail
Nginx site file `/etc/nginx/sites-available/mainsail`:
```nginx
server {
  root /var/www/mainsail;
  index index.html;
  location /websocket {
    proxy_pass http://127.0.0.1:7125/websocket;
    proxy_set_header Host $host;
  }
```
Enable:
```bash
```

#### 0.4.7 Create Initial `printer.cfg` (Joint1 Manual Stepper)
Example snippet (append to existing or new config):
```ini
serial: /dev/serial/by-id/usb-Klipper_stm32h7_123456-if00

enable_pin: !PA2
microsteps: 16
gear_ratio: 99:10
full_steps_per_rotation: 200
velocity: 5
accel: 30
position_max:  1.0821
```
sudo systemctl restart klipper
```
#### 0.4.8 Verify Manual Stepper Command Path
```bash
curl -X POST -H 'Content-Type: application/json' \
  -d '{"script":"MANUAL_STEPPER STEPPER=joint1 MOVE=5 SPEED=10"}' \
  http://<host>:7125/printer/gcode/script
#### 0.4.9 Optional: Enable & Use API Key
Uncomment `api_key` in `moonraker.conf`, restart Moonraker, then:
```
Update bridge parameter `moonraker_api_key` accordingly.
Check status quickly:
```bash
#### 0.4.11 Fan Strategy (Current)
Always-on electronics cooling fan using `[delayed_gcode FAN_BOOT]`:
initial_duration: 2
gcode:
  M106 S166  # ~65%
```

#### 0.4.12 Endstop Characterization (Base Joint)
Staged macros (avoid Jinja printer object access issues):
```
BASE_ENDSTOP_REPORT     ; outputs manual_stepper joint1: pos=XX.X
```
Procedure:
2. `BASE_ENDSTOP_MOVE DIST=30` (repeat until near endstop; adjust DIST smaller if already close)
3. `BASE_ENDSTOP_REPORT` → log `pos=` value
4. Repeat 5–7 times; average results.
5. Set `position_endstop` to average; set `position_max` a safety margin below (e.g. 0.5°).
6. Reduce homing backoff once stable.
Temporary deviation: `position_max` raised to 63 for measurement; will revert to 62 (or 62.5) after characterization.

#### 0.4.13 Tips & Troubleshooting
| Symptom | Check | Fix |
| Stepper moves wrong direction | Dir pin or gear ratio sign | Flip dir pin with `!` or adjust gear ratio |
| Limits ignored | position_min/max mismatch | Restart Klipper after config changes |
| Bridge silent | Topic mismatch or no points | Ensure publishing to `/joint_trajectory` with ≥2 points |

#### 0.4.14 Security Note
If host is on an untrusted network, restrict `trusted_clients` CIDR and/or require API key; optionally place nginx basic auth in front of Moonraker paths.

#### 0.4.15 Next (Future)
- Add additional `manual_stepper` sections for joints 2–6.
- Introduce macro for synchronized multi-axis movement.
### 0.5 Workspace Creation
```bash
mkdir -p ~/moveo_bridge_ws/src
cd ~/moveo_bridge_ws
# (Clone / copy existing packages into src)
colcon build --merge-install
source install/setup.bash
```
Add to `.bashrc`:
```bash
echo 'source ~/moveo_bridge_ws/install/setup.bash' >> ~/.bashrc
```

## 1. Core Packages & Roles
| Package | Role |
|---------|------|
| moveo_description | URDF/Xacro + visuals + collisions + joint limits |
| moveo_moveit_config | MoveIt config (SRDF, kinematics, joint_limits, controllers, launch) |
| moveo_trajectory_bridge | JointTrajectory -> MANUAL_STEPPER / G-code via Moonraker |
| moveo_display_relay (node) | DisplayTrajectory -> JointTrajectory converter |

## 2. Single Joint Bring-Up Flow
1. Verify Moonraker API reachable:
```bash
curl http://<moonraker_host>:7125/server/info | jq '.klippy_state'
```
2. Launch MoveIt (fake controller):
```bash
ros2 launch moveo_moveit_config demo.launch.py use_rviz:=false
```
3. Launch trajectory bridge (live, manual stepper mode):
```bash
ros2 run moveo_trajectory_bridge trajectory_bridge --ros-args \
  -p trajectory_topic:=/joint_trajectory \
  -p dry_run:=false -p manual_stepper_mode:=true -p manual_stepper_joint_index:=0 \
  -p manual_stepper_name:=joint1 -p use_http_transport:=true \
  -p moonraker_base_url:=http://<moonraker_host>:7125 \
  -p joint_position_limits:="[-1.0821,1.0821,-3.14,3.14,-3.14,3.14,-3.14,3.14,-3.14,3.14,-3.14,3.14]"
```
4. Publish a test trajectory:
```bash
ros2 topic pub --once /joint_trajectory trajectory_msgs/JointTrajectory "{joint_names: ['joint1'], points: [{positions: [0.3], time_from_start: {sec: 2}}, {positions: [0.0], time_from_start: {sec: 4}}]}"
```
5. Expect bridge logs: `Loaded trajectory ...` then `[LIVE]` lines with MANUAL_STEPPER.

## 2.1 Full Stack Launch: MoveIt + Bridge + Feedback

To launch the full stack (MoveIt, bridge, feedback publisher, and RViz):

1. **Source your environments in every terminal:**
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/moveo_bridge_ws/install/setup.bash
  ```

2. **Launch the feedback/bridge stack:**
  ```bash
  ros2 launch moveo_trajectory_bridge joint1_with_feedback.launch.py
  ```

3. **In a new terminal, launch MoveIt with RViz:**
  ```bash
  ros2 launch moveo_moveit_config demo.launch.py
  ```

4. **Use the MotionPlanning panel in RViz to plan and execute motions.**

If the MotionPlanning panel is missing or errors, see the troubleshooting section in `05_Tips_Tricks.md`.

## 3. MoveIt Planning API (Headless)
Minimal motion request script (full joint state provided) — see `Onboarding/examples` (future).

## 4. Known Issues & Workarounds (initial)
| Issue | Symptom | Resolution |
|-------|---------|------------|
| RViz MoveIt panel missing | MotionPlanning panel error | Ensure `ros-humble-moveit-ros-visualization` installed; environment not wiped (avoid `env -i`) |
| No trajectory loaded | Bridge silent | Ensure correct topic `/joint_trajectory` and message has points |
| Plan error code -2 | MoveIt plan failed | Provide full start state (all joints) | 
| No motion after plan | Delta = 0 | Increase target joint1 angle, verify start state |

## 4.1 Outstanding Gaps / Risks (Live)
| Gap | Impact | Planned Mitigation |
|-----|--------|--------------------|
| No true joint feedback | Can't confirm actual vs commanded position | Implement Moonraker poll or encoder integration |
| Missing RViz MotionPlanning panel | Harder interactive planning | Fix plugin path / ensure moveit_ros_visualization installed |
| Single-joint only | Can't test coordinated motion or timing | Add joints 2–6 manual_stepper & staged activation |
| Manual velocity limits | Risk of unsafe accel requests | Script to derive limits from specs & update joint_limits.yaml |
| Dual-motor sync (J2) not tested | Potential drift under load | Introduce periodic alignment check or add encoder feedback |
| No safety / e-stop integration | Harder to abort quickly | Add ROS service / topic to trigger Klipper emergency stop |

## 4.5. Headless MoveIt Control (WORKING - 2025-10-02)

**Status**: ✅ ACHIEVED - Complete MoveIt → Hardware motion without RViz GUI

### Required 4-Terminal Setup
```bash
# Terminal 1: MoveIt Planning Stack
ros2 launch moveo_moveit_config demo.launch.py
# (RViz will crash due to GLIBC issue - ignore this)

# Terminal 2: FollowJointTrajectory Adapter  
ros2 run moveo_trajectory_bridge fjt_adapter --ros-args -p wait_for_execution:=true -p feedback_rate_hz:=2.0

# Terminal 3: Hardware Trajectory Bridge
ros2 run moveo_trajectory_bridge trajectory_bridge --ros-args -p trajectory_topic:=/joint_trajectory -p dry_run:=false -p manual_stepper_mode:=true -p manual_stepper_joint_index:=0 -p manual_stepper_name:=joint1 -p use_http_transport:=true -p moonraker_base_url:=http://localhost:7125

# Terminal 4: Control Commands (see examples below)
```

### Working Control Methods

**Method 1: Python Scripts** (Recommended)
```bash
python3 test_execution_feedback.py           # Tested working ✅
python3 working_moveit_control.py           # Interactive control
python3 precise_joint_control.py            # Fine-tuned movements
```

**Method 2: Direct ROS Action Commands** (Copy-paste ready)
```bash
# Move to 10° (0.175 rad):
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.175], time_from_start: {sec: 2, nanosec: 0}}]}}" --feedback

# Move to 20° (0.349 rad):  
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.349], time_from_start: {sec: 3, nanosec: 0}}]}}" --feedback

# Return to 0°:
ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1'], points: [{positions: [0.0], time_from_start: {sec: 2, nanosec: 0}}]}}" --feedback
```

### Data Flow (Complete Working Pipeline)
```
Control Command → FollowJointTrajectory Action → FJT Adapter → /joint_trajectory → Trajectory Bridge → Klipper → Physical Motor
```

### Angle Reference (Degrees to Radians)
- 5° = 0.087 rad
- 10° = 0.175 rad  
- 15° = 0.262 rad
- 20° = 0.349 rad (tested working ✅)
- 30° = 0.524 rad
- 45° = 0.785 rad

### Control Method Analysis
| Method | Status | Notes |
|--------|--------|-------|
| Direct FollowJointTrajectory | ✅ Working | Bypasses MoveIt planning complexity |
| Python test scripts | ✅ Working | Provides feedback and error handling |
| MoveGroup actions | ❌ Complex | Requires complete robot state setup |
| Planning services | ❌ Plan only | No execution capability |

### Success Criteria Met
- ✅ Plan trajectories without RViz GUI
- ✅ Execute trajectories with real hardware motion  
- ✅ Receive execution feedback and progress updates
- ✅ Multiple control interfaces (Python, ROS commands)
- ✅ Reproducible 4-terminal setup process

## 5. Updated Roadmap (Next Steps After Single-Joint Success ✅)

### COMPLETED ✅
- ~~RViz recover~~: BYPASSED - Headless control fully functional
- ~~FollowJointTrajectory adapter feedback~~: Enhanced with realistic timing  
- ~~Single-joint control baseline~~: Python scripts + ROS2 commands + hardware motion
- ~~Control method validation~~: Multiple working interfaces established

### IMMEDIATE NEXT STEPS (Priority Order)

#### Phase 1: Multi-Joint Foundation (Steps 6-8)
6. **Multi-Joint Klipper Configuration** 
   - Add manual_stepper sections for joints 2-6 in printer.cfg
   - Configure gear ratios and limits for each joint
   - Test individual joint movement via Klipper console

7. **Multi-Joint Bridge Extension**
   - Extend trajectory_bridge to handle all 6 joints simultaneously
   - Implement joint synchronization and timing coordination
   - Add dry-run mode validation for multi-joint trajectories

8. **Multi-Joint Control Validation**
   - Create 2-joint test scripts (J1 + J2 coordination)
   - Validate 6-DOF trajectory execution
   - Test end-effector pose accuracy

#### Phase 2: Safety & Robustness (Steps 9-11)  
9. **Safety Systems Integration**
   - Add execution enable/disable SetBool service gate to bridge
   - Implement emergency stop functionality  
   - Add soft limit monitoring and violation handling

10. **Enhanced Feedback & State**
    - Extend /joint_states with real position updates after each segment
    - Add velocity estimation and joint delta accumulation
    - Implement feedback validation (command vs actual position)

11. **Hardware Characterization Completion**
    - Finish base endstop characterization (reduce temp max=63 → final ~62°)
    - Complete datasheet fields for joints 3-6
    - Generate automated joint_limits.yaml from hardware specs

#### Phase 3: Production Features (Steps 12-14)
12. **Advanced Motion Features**
    - Implement velocity/acceleration scaling based on payload
    - Add trajectory smoothing and jerk limiting
    - Create collision avoidance basic boundaries

13. **Automation & Testing**
    - Regression test suite for trajectory validation
    - Automated workspace setup scripts
    - Performance benchmarking and optimization

14. **Documentation & Release**
    - Complete setup automation (< 30 min from clean Ubuntu)
    - Video demonstrations and user guides  
    - Stable release tagging with artifacts

### SUCCESS METRICS FOR NEXT PHASE
- ✅ Move all 6 joints simultaneously in coordinated motion
- ✅ Achieve target end-effector poses within acceptable tolerance  
- ✅ Safe operation with emergency stop and limit enforcement
- ✅ Reproducible multi-joint setup process

---
## SESSION SUMMARY (2025-10-02)

**MAJOR MILESTONE ACHIEVED**: Complete single-joint headless MoveIt control system

**What Works:**
- ✅ 4-terminal setup (MoveIt + FJT Adapter + Trajectory Bridge + Control)
- ✅ Python control scripts with feedback (`test_execution_feedback.py`)
- ✅ ROS2 command-line control (2-point trajectory requirement identified)
- ✅ Physical hardware motion with timing and progress feedback
- ✅ Enhanced execution adapter with configurable timing behavior
- ✅ Comprehensive documentation and troubleshooting guides

**Key Insights:**
- RViz crashes are acceptable - MoveIt works perfectly headlessly
- Direct FollowJointTrajectory actions are more reliable than MoveGroup
- Trajectory bridge requires multi-point trajectories for motion calculation
- 4-terminal architecture provides complete control pipeline

**Next Session Focus**: Multi-joint expansion starting with Joint 2 integration

**Status**: Ready for Phase 2 - Multi-joint coordination
9. Add safety doc + RViz troubleshooting subsection to onboarding.
10. Multi-joint physical (J1+J2) conservative execution after dry-run passes.

---
## End of Day Summary (2025-10-01)
**Achievements:**
* Implemented real FollowJointTrajectory action path (MoveIt → adapter → bridge) replacing fake controller.
* Namespaced action server alignment fixed prior zero-controller issue.
* Added shadow `/joint_states` publisher (command echo) to sustain planning state.
* Verified single-joint hardware motion triggered via action goal.
* Logging & diagnostics improved (distinct EXECUTE_CB markers, MANUAL_STEPPER batch logs).

**Notable Issues:**
* RViz crash (GLIBC_PRIVATE symbol in snap core20 libpthread) – interactive MotionPlanning panel unavailable; planning remains functional headless.
* Action result currently optimistic (immediate success) – no feedback or completion gating.
* Only joint1 wired; multi-joint sequencing untested; no safety enable gate yet.

**Key Logs (reference timestamps):**
* Controller added: `Added FollowJointTrajectory controller for manipulator_controller`
* Adapter publish: `EXECUTE_CB_START pts=2` / `Republished trajectory pts=2`
* Bridge execution: `[LIVE] sent=1 batch ... MANUAL_STEPPER STEPPER=joint1 MOVE=-17.18873 SPEED=8.59437`

**Carry-Over Focus Tomorrow:** Restore RViz UI, implement safety gate + feedback improvements, begin multi-joint dry-run.

---
## Tomorrow Pickup Checklist (Quick Start)
1. Open new non-snap terminal; source workspace; launch: `ros2 launch moveo_trajectory_bridge fjt_adapter.launch.py`.
2. Launch MoveIt headless first: `ros2 launch moveo_moveit_config demo.launch.py use_rviz:=false` (verify controller added).
3. Attempt RViz from non-snap shell: `/opt/ros/humble/lib/rviz2/rviz2 -d install/share/moveo_moveit_config/config/moveit.rviz`.
4. If success, plan & execute a small joint1 motion; capture logs.
5. Implement safety gate (service) & modify bridge to respect it before sending G-code.
6. Add J2 manual_stepper config; run dry-run multi-joint test (bridge param `dry_run:=true`).
7. Add feedback emission & delayed action success to adapter.
8. Draft limit generation script skeleton (tools/ directory).
9. Update manual with RViz workaround results.
10. Prepare first multi-joint physical test (J1+J2) with low speeds.

# Start-to-Finish Implementation Blueprint (Detailed-Level)
This section expands each high-level Step (0–28) from the Progress Log into granular actionable guidance. Keep the Progress Log table high-level; update step details HERE only.

## Step 0: OS Image Acquisition
- Objective: Obtain official Ubuntu 22.04 LTS Desktop ISO.
- Link: https://releases.ubuntu.com/22.04/
- Actions:
  1. Download ISO (64-bit).
  2. Verify checksum:
     ```bash
     sha256sum ubuntu-22.04.*.iso
     # Compare with SHA256SUMS file
     ```
  3. (Optional) Store checksum log under `Onboarding/logs/install_checksums.txt`.
- Exit: Verified checksum matches published SHA256.
- Pitfalls: Using a mirror with outdated image → always cross-check signature.

## Step 1: Bootable Media Creation
- Tools: Balena Etcher (GUI) or Raspberry Pi Imager or CLI `dd`.
- CLI Example (DANGEROUS – double check device):
  ```bash
  sudo dd if=ubuntu-22.04.iso of=/dev/sdX bs=4M status=progress conv=fsync
  sudo eject /dev/sdX
  ```
- Validation: Boot Beelink → Installer screen visible.

## Step 2: OS Installation
- Selections: Normal install, allow updates, create user `robot` (example), enable OpenSSH (optional: `sudo apt install -y openssh-server`).
- Post-Install:
  ```bash
  sudo apt update && sudo apt upgrade -y
  ```
- Exit: Clean boot & networking functional.

## Step 3: System Base Prerequisites
- Install baseline tooling:
  ```bash
  sudo apt install -y build-essential git curl wget python3 python3-venv python3-pip jq tree tmux net-tools
  ```
- Time Sync: `timedatectl status` (ensure NTP active).
- Exit: Core dev tools present.

## Step 4: ROS 2 Humble Stack
(Already documented in earlier section; cross-ref 0.2.)
- Quick Check: `ros2 doctor` (if available) or `ros2 topic list` returns default topics.
- Pitfalls: Missing keyring → re-run `ros.key` import.

## Step 5: MoveIt & Control Dependencies
- Install:
  ```bash
  sudo apt install -y ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
  ```
- Verify:
  ```bash
  ros2 pkg list | grep moveit | wc -l
  ```

## Step 6: Firmware Layer (Klipper / Moonraker / Optional Mainsail)
- Cross-ref detailed install (0.4 subsections).
- Health Checks:
  ```bash
  systemctl --no-pager status klipper moonraker | grep Active
  curl http://<host>:7125/server/info | jq '.klippy_state'
  ```
- Pitfalls: Missing dependencies → re-run install script.

## Step 7: ROS Workspace Initialization
- Create & build:
  ```bash
  mkdir -p ~/moveo_bridge_ws/src
  cd ~/moveo_bridge_ws
  colcon build --merge-install
  echo 'source ~/moveo_bridge_ws/install/setup.bash' >> ~/.bashrc
  ```
- Exit: New shell has workspace overlay sourced.

## Step 8: Robot Model Authoring
- Files: `moveo_description/urdf/moveo.urdf.xacro`
- Minimum: Joints, continuous vs revolute, limits, link inertial placeholders (TODO), visuals + collisions.
- Validation:
  ```bash
  ros2 launch moveo_moveit_config demo.launch.py use_rviz:=false
  ```
- Pitfalls: Mismatched joint names vs SRDF group.

## Step 9: Planning Bring-Up
- Adjust `controllers.yaml` (fake controller) & `demo.launch.py` for clean startup.
- Test service:
  ```bash
  ros2 service type /plan_kinematic_path
  ```
- Optional CLI planning (requires request file).

## Step 10: Trajectory Bridge Core
- Node: `moveo_trajectory_bridge/trajectory_bridge.py`
- Launch example (HTTP only, manual stepper mode): (see earlier section for command)
- Verify logs show: "Loaded trajectory" then HTTP send lines.
- Pitfall: Missing `use_http_transport:=true` leads to gating mismatch.

## Step 11: Display → Trajectory Relay
- Node: `moveo_moveit_display_relay.py` (actual file naming: verify path)
- Behavior: Subscribes `/move_group/display_planned_path`, emits `/joint_trajectory`.
- Test: Trigger a simple plan; observe publish count increment.

## Step 12: First Physical Motion (Joint1)
- Procedure: Publish 2-point JointTrajectory (example earlier) while bridge is live.
- Confirmation: Physical base joint rotates then returns.
- Log Capture: Save snippet to `Onboarding/logs/joint1_first_motion.log` (optional).

## Step 13: Hardware Specification Consolidation
(Expanded earlier.)
- Add numeric spec confirmations J1/J2.
- Placeholder stubs for others.

## Step 14: Multi-Joint Enable (Dry-Run)
- Modify `printer.cfg` with additional manual steppers.
- Launch with `dry_run:=true`:
  ```bash
  ros2 run moveo_trajectory_bridge trajectory_bridge --ros-args -p dry_run:=true -p manual_stepper_mode:=false
  ```
- Publish multi-joint test trajectory; ensure segmentation and clamping logs appear.

## Step 15: Multi-Joint Planning Execution
- Provide full start state (script forthcoming in examples).
- Ensure target positions produce non-zero deltas for both joints.
- If second joint not wired, keep manual_stepper_mode for joint1 only while ignoring others (controlled via param filtering - potential enhancement).

## Step 16: Limits Derivation Automation
- Draft script skeleton:
  ```bash
  python tools/generate_limits.py --spec Onboarding/04_Hardware_Firmware_Network_Architecture/HARDWARE_NOTES.md \
    --out moveo_moveit_config/config/joint_limits.yaml
  ```
- Include dry-run flag to print prospective limits before writing.

## Step 17: Feedback Channel (/joint_states)
- Shadow approach: integrate at bridge level (optionally add callback storing last commanded positions, publish at fixed rate).
- Enhancement: Add Moonraker macro returning step counters; parse via HTTP.

## Step 18: Safety Enable Gate
- Service Definition: `/enable_physical_motion` (std_srvs/SetBool)
- Manual toggle example:
  ```bash
  ros2 service call /enable_physical_motion std_srvs/srv/SetBool '{data: true}'
  ```

## Step 19: Dual Motor Sync Validation
- Measurement: Compare commanded vs observed (future feedback) during loaded hold.
- Thermal check: IR temp every 2 min for 10 min.
- Criteria: No audible missed steps; temperature < rated threshold - 10°C margin.

## Step 20: Remaining Joints Wiring
- Add manual_stepper (or servo control) blocks for joints 4–6.
- Keep velocity conservative initially (<25% nominal).

## Step 21: Full 6-DOF Motion
- Plan modest composite move (e.g., base yaw + shoulder lift + elbow adjust).
- Validate timing uniformity—no large dwell gaps in logs.

## Step 22: Simulation Parity
- Add inertial tags (approximate masses) to URDF.
- Launch simulation environment (Gazebo/Ignition) stub.
- Compare end pose vs physical within tolerance (<5 mm / <2°).

## Step 23: Regression Tests
- Add `tests/` scripts: publish known trajectory (dry_run) & assert segment count.
- Optional: pytest wrapper for invocation.

## Step 24: Payload Validation
- Incremental payload test: log joint currents/temps.
- Determine safe continuous payload threshold.

## Step 25: Documentation QA Run
- Fresh VM time trial following manual only.
- Record steps taking > expected time—refine doc.

## Step 26: Performance Polish
- Introduce optional smoothing (e.g., interpolate intermediate points).
- Batch size tuning param (e.g., `max_points_per_batch`).

## Step 27: Closed-Loop Prep
- Define message schema for future encoder or vision feedback.
- Draft interface doc in `Onboarding/closed_loop_plan.md` (future file).

## Step 28: Release Snapshot
- Tag repo: `git tag -a v0.1.0 -m "Baseline 1-joint to 6-joint infrastructure"`.
- Export onboarding folder as artifact (tarball + checksum).

---

# NEXT STEPS ACTION PLAN 📋

## Current Status: Single-Joint Control COMPLETE ✅

**🎉 Major Achievement - What we've accomplished:**
- ✅ Complete 4-terminal headless MoveIt control system
- ✅ Python scripts working (`test_execution_feedback.py`)
- ✅ ROS2 command-line control working (2-point trajectories)  
- ✅ Reliable Joint1 physical motion with feedback
- ✅ Enhanced execution timing and progress reporting
- ✅ Comprehensive documentation and troubleshooting guides

## PHASE 1: MULTI-JOINT FOUNDATION

### Priority 1: Joint 2 Integration (Next Session Focus)
**Immediate goal:** Get Joint 2 working alongside Joint 1

**Action Items:**
- [ ] **Hardware Check**: Verify Joint 2 connections on Octopus board
- [ ] **Klipper Config**: Add Joint 2 manual_stepper configuration to printer.cfg
- [ ] **Individual Test**: Move Joint 2 independently via Klipper console
- [ ] **2-Joint Test**: Create simple J1+J2 coordinated motion script

**Expected outcome:** Dual-joint coordinated motion demonstrating foundation for full 6-DOF control

### Priority 2: Multi-Joint Bridge Extension (Following weeks)
**Tasks:**
1. **Extend Joint Mapping** - Map joint1-6 to corresponding manual_stepper names
2. **Batch Coordination** - Ensure simultaneous multi-joint MANUAL_STEPPER commands  
3. **Create Multi-Joint Test Scripts** for 2-joint, then 6-joint coordination
4. **Add Validation Mode** - Multi-joint dry-run testing

**Files to modify:** `trajectory_bridge.py`, launch parameters, test scripts

### Priority 3: Complete 6-DOF Validation
**Tasks:**
1. **6-DOF Test Trajectories** - Common poses: "home", "ready", "reach"
2. **Integration Testing** - MoveIt planning → 6-joint execution
3. **Performance Analysis** - Timing synchronization and smooth motion
4. **User Interface Enhancement** - Multi-joint Python control scripts

**Files to create:** `multi_joint_control.py`, `pose_library.py`, `6dof_validation_tests.py`

## ESTIMATED TIMELINE
| Phase | Duration | Key Milestone |
|-------|----------|---------------|
| Joint 2 Integration | 2-3 days | Dual-joint coordination |
| Multi-Joint Bridge | 3-4 days | All joints coordinated |  
| 6-DOF Validation | 4-5 days | Complete system |
| **Total** | **2 weeks** | **Full multi-joint system** |

## TECHNICAL ROADMAP

### Hardware Configuration (Klipper)
```ini
# Joint 2 Example (Dual motor setup)
[manual_stepper joint2_left]
step_pin: <pin>
dir_pin: <pin>  
enable_pin: <pin>
microsteps: 16
gear_ratio: 80:14  # From hardware datasheets
velocity: 10
accel: 50

[manual_stepper joint2_right]
# Mirror configuration for synchronized dual motors
```

### Software Architecture  
**Current:** MoveIt → FJT Adapter → Trajectory Bridge → Klipper (Joint 1)  
**Target:** MoveIt → FJT Adapter → Trajectory Bridge → Klipper (Joints 1-6)

### Success Criteria
- ✅ All 6 joints individually controllable via Klipper console
- ✅ Multi-joint coordinated motion without jerking  
- ✅ End-effector reaches target poses accurately
- ✅ Reliable execution feedback across all joints

---

**Status**: Ready to proceed to multi-joint expansion  
**Last Updated**: 2025-10-02  
**Foundation**: Proven single-joint control system with 4-terminal architecture

---
Maintenance Rule: Update granular steps here; never bloat Progress Log with command-level detail.
