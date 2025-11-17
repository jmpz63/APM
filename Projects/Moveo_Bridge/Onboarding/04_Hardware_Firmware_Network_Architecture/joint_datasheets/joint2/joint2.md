# Joint 2 Datasheet Summary (Dual Motor Shoulder)

Source assets:
- Motor STEP: `../../src/moveo_trajectory_bridge/datasheets/joint2/23HS32-4004S.STEP`
- Motor PDF: `../../src/moveo_trajectory_bridge/datasheets/joint2/23HS32-4004S_Full_Datasheet.pdf`
- Torque Curve PDF: `../../src/moveo_trajectory_bridge/datasheets/joint2/23HS32-4004S_Torque_Curve.pdf`
- Pulley Tooth Photo: `../../src/moveo_trajectory_bridge/datasheets/joint2/Joint2 teeth count.jpeg`

## Core Specs (Fill / Verify)
- Motor Model: 23HS32-4004S (x2 parallel on same axis)
- Native Steps/Rev: 200
- Microstep Setting: 4 (verify)
- Belt / Gear Reduction: 80:14 (5.714:1) (confirm from photo & tooth counts)
- Effective Steps / Joint Rev: 200 * 4 * 5.714 ≈ 4571 steps
- Rated Current (A/phase): 4.0 A each — Derate strategy? (e.g. 65% continuous)
- Combined Holding Torque: 240 N·cm each → 480 N·cm ideal (minus coupling inefficiencies)
- Rotor Inertia (g·cm²): (fill)
- Phase Resistance (Ohms): (fill)
- Inductance (mH): (fill)
- Max Practical Motor RPM: (enter) → Raw Joint Deg/s calc pending
- Proposed Planning Vel Limit: 25–30 deg/s (due to high load + inertia)
- Proposed Planning Accel Limit: 120–140 deg/s²

## Load / Torque Notes
Reference payload torque analysis in `HARDWARE_NOTES.md` (shoulder worst-case overhead reach). Ensure gravitational torque < 60% available continuous torque.

## Dual Motor Synchronization
- Current strategy: Both manual_stepper definitions (joint2 & joint2b) commanded identically.
- Future: implement drift monitor (encoder or step discrepancy detection if feedback added).

## Pending Data
- Weight & CoM of upper links + payload test mass
- Thermal rise after 5 min static hold at shoulder 45° loaded

## Risk / Observations
- High current drivers may approach thermal limits without active cooling.
- Long-term: consider reduction increase if torque margin proves insufficient.
