# (Imported) Moveo Arm Hardware Notes

> Added pointer: See `joint_datasheets/` for per-joint extracted specs.
> Board: BigTreeTech Octopus Max EZ (MCU: STM32H723ZET6) – details in `board/Octopus_Max_EZ.md`.

Source copy original path: `src/moveo_trajectory_bridge/HARDWARE_NOTES.md`

> This file is duplicated here for centralized onboarding reference. Update THIS copy going forward; if bridge package copy diverges we will reconcile.

---

# Moveo Arm Hardware Notes (Actuators & Transmission)

Populate this file with the actual motor + transmission data per joint. These values drive: 
- Joint velocity & acceleration limits (planning + runtime scaling)
- Feedrate heuristic sanity checking
- Torque margin vs gravity / dynamic load
- Thermal / current derating strategies
- Dual-motor synchronization (Joint 2)
- Future: effort interface emulation, collision avoidance tuning, jerk limiting

## Quick Instructions
Fill in each joint section. If a field is not applicable, mark `n/a`. After completing, we can script a helper to auto-generate conservative velocity / acceleration caps and update `joint_limits.yaml` + runtime `joint_velocity_caps`.

Datasheets folder: `joint_datasheets/` (add PDFs/images there; see README for naming suggestions).

## Global Controller / Firmware Parameters
- MCU / Board: (fill)
- Firmware: Klipper (version / commit): (fill)
- Supply Voltage (V): (fill)
- Ambient Typical (°C): (fill)
- Max Allowable Motor Surface Temp (°C): (e.g. 70)
- Safety Derate (%) for current: (e.g. 70%)

## Joint Summary Table (Fill Values Then Details Below)
| Joint | Axis Description | Motor Model | Native Steps/Rev | Microstep | Gear Ratio (motor:joint) | Effective Steps/Joint Rev | Max Rated Current (A/phase) | Holding Torque (N·cm) | Rotor Inertia (g·cm²) | Max Motor RPM | Est. Max Joint Deg/s | Proposed Vel Limit Deg/s | Proposed Accel Deg/s² |
|-------|------------------|-------------|------------------|-----------|--------------------------|---------------------------|----------------------------|-----------------------|-----------------------|---------------|----------------------|--------------------------|------------------------|
| J1    | Base yaw         | 17HS24-0644S | 200              | 4         | 10:1                     | 8000                      | 0.6                        | (~40 TBD)             | (TBD)                | 300          | (calc)               | 60                       | 300                    |
| J2    | Shoulder pitch (dual) | 23HS32-4004S x2 | 200 | 4 | ~80:14 (5.714:1)          | ~4571                     | 4.0                        | 240 (ea)               | (TBD)                | (TBD)        | (calc)               | 25–30                    | 120–140                |
| J3    | Elbow pitch      |             |                  |           |                          |                           |                            |                       |                       |               |                      |                          |                        |
| J4    | Wrist pitch      |             |                  |           |                          |                           |                            |                       |                       |               |                      |                          |                        |
| J5    | Wrist roll       |             |                  |           |                          |                           |                            |                       |                       |               |                      |                          |                        |
| J6    | Tool rotation    |             |                  |           |                          |                           |                            |                       |                       |               |                      |                          |                        |

(Continue populating as hardware characterization proceeds.)

---

## Per Joint Detailed Sections

### Joint 1 (Base Yaw)
(see original detailed section in source file for formulas – mirror edits here)

### Joint 2 (Shoulder Pitch - Dual Motors)
(see original detailed section in source file for formulas – mirror edits here)

### Joint 3 ... etc.

---

## Derived Calculations / Scripts (Planned)
- Autogenerate `joint_velocity_caps` from filled table.
- Update `moveo_moveit_config/config/joint_limits.yaml` programmatically.
- Produce torque margin report for payload scenarios.

## Action Items
1. Finish table baseline for J1 & J2 (complete datasheet confirmation).
2. Add raw mass & link length measurements.
3. Compute static torque tables vs reach for J2 & J3.
4. Integrate calculation script.

---
Last sync: 2025-10-01 (Added: always-on fan (65%), removed conditional fan macros, base endstop characterization macros staged (position_max temporarily 63 for measurement); joint6 datasheet stub present; pending population J3–J6 specs & velocity derivation script.)
