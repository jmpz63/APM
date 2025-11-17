# Joint X Datasheet Template

Copy this template to create or expand each `jointX.md`.

## Source Assets
List relative links to PDFs, STEP, images under `../../src/moveo_trajectory_bridge/datasheets/jointX/`.

## Core Specs
- Motor Model:
- Native Steps/Rev:
- Microstep Setting:
- Driver Type & Mode (e.g. TMC5160, spreadCycle/stealthChop):
- Gear / Belt Ratio (motor:joint):
- Effective Steps / Joint Rev (calc):
- Rated Current (A/phase):
- Recommended Continuous Current (A/phase) (derated):
- Holding Torque (N·cm):
- Rotor Inertia (g·cm²):
- Phase Resistance (Ohms):
- Inductance (mH):
- Max Practical Motor RPM (derated):
- Raw Max Joint Deg/s (calc):
- Proposed Planning Vel Limit Deg/s:
- Proposed Planning Accel Limit Deg/s²:
- Backlash Estimate (deg):
- Static Friction Breakaway Current (% rated):

## Transmission Details
Describe pulley tooth counts, gearbox stages, belt type (e.g. GT2 6mm), lubrication/maintenance notes.

## Thermal / Reliability Notes
Ambient test conditions, temperature rise, safe cap.

## Validation Checklist
- [ ] Pulley tooth counts photographed
- [ ] Torque curve inspected for derating
- [ ] Accel limit validated via step-loss test
- [ ] Current tuned & verified stable

## Change Log
- (date) - initial entry
