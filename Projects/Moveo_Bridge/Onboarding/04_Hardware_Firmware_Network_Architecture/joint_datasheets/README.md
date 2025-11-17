# Joint Datasheets (Integrated Index)

This directory provides an indexed and curated reference to all actuator & transmission datasheets originally located under:
`src/moveo_trajectory_bridge/datasheets/`

We DO NOT duplicate large binary assets (PDF, STEP, images) to avoid repo bloat / divergence. Instead we:
- Link to the original relative path assets.
- Summarize key extracted parameters (when available).
- Track a hash (future automation) to detect spec changes.

## Structure
```
joint_datasheets/
  README.md (this file)
  joint1.md
  joint2.md
  joint3.md
  joint4.md
  joint5.md
  joint6.md
  index_parameters.json (future automation placeholder)
```

## Contribution Workflow
1. Add or update raw datasheet assets in `src/moveo_trajectory_bridge/datasheets/...`.
2. Extract key specs (steps/rev, current, resistance, inductance, torque curve notable points, gear ratio) into the corresponding `jointX.md` file here.
3. Update `HARDWARE_NOTES.md` summary table with any changed effective values.
4. (Future) Run the limits generation script to refresh planning/runtime caps.

## Key Parameter Set (Target For Each Joint)
- Motor Model
- Electrical: Rated Current (A/phase), Resistance (Ohms), Inductance (mH)
- Mechanical: Holding Torque (N·cm), Detent Torque, Rotor Inertia (g·cm²)
- Gear / Belt Transmission: Stage(s), ratio, tooth counts
- Microstepping & Driver Mode
- Effective Steps per Joint Revolution
- Max Practical Motor RPM (derated)
- Calculated Max Joint Deg/s (pre-limit)
- Recommended Velocity / Accel Limit
- Thermal / Continuous Current Safe %
- Notes / Caveats

## Future Automation
A script will:
- Hash each binary asset
- Generate `index_parameters.json`
- Flag differences to prompt limit recomputation

