# Joint 5 Datasheet Summary (Wrist Roll)

Source assets:
- (Add) e.g. `../../src/moveo_trajectory_bridge/datasheets/joint5/motor.pdf`

## Core Specs (Placeholder)
- Motor Model: (fill)
- Native Steps/Rev: 200
- Microstep Setting: (fill)
- Gear / Belt Ratio: 44:10  (note: set in printer.cfg as gear_ratio: 44:10)
- Effective Steps / Joint Rev: (calc)
- Rated Current (A/phase): (fill)
- Holding Torque (N·cm): (fill)
- Rotor Inertia (g·cm²): (fill)
- Max Practical Motor RPM: (fill)
- Raw Max Joint Deg/s: (calc)
- Proposed Planning Vel Limit: (placeholder 120 deg/s)
- Proposed Planning Accel Limit: (placeholder 400 deg/s²)

## Notes
High speed rotation potential; ensure torque still adequate for tool inertia.

Gear Ratio: 44:10 confirmed. See also printer.cfg [manual_stepper joint5] gear_ratio.
