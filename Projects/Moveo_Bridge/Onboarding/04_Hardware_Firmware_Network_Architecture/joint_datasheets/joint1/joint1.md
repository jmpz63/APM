# Joint 1 Datasheet Summary

Source assets (do not duplicate):
- STEP: `../../src/moveo_trajectory_bridge/datasheets/joint1/17HS24-0644S.STEP`
- Motor PDF: `../../src/moveo_trajectory_bridge/datasheets/joint1/17HS24-0644S_Full_Datasheet.pdf`
- Torque Curve: `../../src/moveo_trajectory_bridge/datasheets/joint1/17HS24-0644S_Torque_Curve.pdf`
- Klipper Template: `../../src/moveo_trajectory_bridge/datasheets/joint1/Klipper_J1_template.cfg`

## Extracted Core Specs (Fill / Verify)
- Motor Model: 17HS24-0644S
- Native Steps/Rev: 200
- Microstep Setting: 4 (verify in `printer.cfg` driver config)
- Gear Ratio: 10:1 (confirm tooth counts)
- Effective Steps / Joint Rev: 200 * 4 * 10 = 8000
- Rated Current (A/phase): 0.64 A (spec nominal) — derate target ~0.55 A
- Holding Torque (N·cm): (enter from datasheet, ~40?)
- Rotor Inertia (g·cm²): (fill)
- Phase Resistance (Ohms): (fill)
- Inductance (mH): (fill)
- Max Motor RPM Practical: 300 (est) → Joint deg/s raw = (300 rev/min * 360 deg/rev / 60) / 10
- Raw Max Joint Deg/s ≈ (300 * 360 / 60) / 10 = 180 deg/s (pre-derate)
- Proposed Planning Vel Limit: 60 deg/s
- Proposed Planning Accel Limit: 300 deg/s²
- Thermal Notes: Base enclosed? add airflow note.

## Pending Measurements
- Backlash estimate (deg)
- Static friction holding current threshold

## Notes
- Validate if 10:1 is compound (multi-stage) or single belt reduction.
- Add photo of belt/pulley stage to datasheets directory if missing.
