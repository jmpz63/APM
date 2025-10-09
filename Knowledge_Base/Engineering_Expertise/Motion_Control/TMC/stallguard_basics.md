# StallGuard (Sensorless) Basics and Setup for TMC5160

This guide summarizes practical steps to configure, tune, and operate StallGuard in Klipper with TMC5160 drivers. It distills the official procedures and adds robotics-focused tips.

Important: StallGuard only works reliably in SpreadCycle. Do not attempt to tune in StealthChop.

## When to use
- Crash/limit detection for axes where physical endstops are impractical.
- Detect unexpected contact for safety interlocks. Beware: not a substitute for primary E‑stop and physical hard limits.

## Prerequisites
- TMC5160 wired via SPI with DIAG pin connected to MCU input.
- Axis mechanics can safely bump into a hard stop during tuning without damage.
- Stepper configuration checked (steps, rotation_distance, current, direction, etc.).

## Klipper configuration sketch

```
[tmc5160 stepper_jointX]
cs_pin: ...
diag1_pin: ^!PA1          # or diag0_pin depending on your board wiring
run_current: 1.6          # start conservative; ensure thermals are OK
interpolate: False
# Keep SpreadCycle (no stealthchop_threshold)

[stepper_jointX]
endstop_pin: tmc5160_stepper_jointX:virtual_endstop
homing_retract_dist: 0    # required for sensorless homing
homing_speed: <~rot_dist/2>  # roughly 1 rev every 2s as a starting point
```

Notes
- Do not set hold_current during tuning.
- Ensure no gcode/macros switch modes or current during homing.

## Tuning procedure (condensed)
Follow Klipper’s six-step flow; here’s the abbreviated field commands for TMC5160:

1) Place carriage mid‑travel; ensure SpreadCycle active.
2) Set maximum sensitivity and test that motion halts immediately:
   - `SET_TMC_FIELD STEPPER=stepper_jointX FIELD=sgt VALUE=-64`
   - `G28 X0` (or appropriate axis)
   - If it moves far without halting, wiring/pin is wrong; stop and fix.
3) Bisect upwards (toward 63) to find the highest sensitivity that still homes reliably end‑to‑end. Record as max_sensitivity.
4) Continue upwards to find the lowest sensitivity that still homes with a single, non‑banging touch. Record as min_sensitivity.
5) Compute final SGT ≈ min + (max − min)/3; set in `[tmc5160]` as `driver_SGT: <value>`.
6) Create/adjust macros to:
   - Pause ≥2s before homing (clear stall flag).
   - Lower/raise current if needed around homing.
   - Immediately move away from rail after homing.

Example helper macro

```
[gcode_macro SENSORLESS_HOME_JOINTX]
gcode:
    {% set HOME_CUR = 0.8 %}
    {% set cfg = printer.configfile.settings['tmc5160 stepper_jointX'] %}
    {% set RUN_CUR = cfg.run_current %}
    SET_TMC_CURRENT STEPPER=stepper_jointX CURRENT={HOME_CUR}
    G4 P2000
    G28 X0
    G90
    G1 X5 F1200
    SET_TMC_CURRENT STEPPER=stepper_jointX CURRENT={RUN_CUR}
```

## Practical limits and caveats
- Very slow speeds (<10 RPM) generate little back‑EMF; stalls may be missed.
- Very high speeds approach supply back‑EMF; stalls can be missed.
- Temperature and current affect sensitivity: re‑tune if these change materially.
- Leadscrews can generate large forces; verify mechanics can withstand homing bumps.

## Troubleshooting quick hits
- No stall ever: wrong diag pin, pull‑ups, or endstop pin mapping; or still in StealthChop.
- Banging before stop: increase homing speed slightly or raise current modestly; then re‑find window.
- Intermittent misses: widen window by increasing speed or current; confirm wiring integrity.
- Overtemp/otpw: reduce current, add cooling, or lower duty cycle of homing attempts.

## References
- Klipper: TMC Drivers → Sensorless homing section
- Klipper: Config Reference → `[tmc5160]`, SET_TMC_FIELD, DUMP_TMC
- Trinamic TMC5160 datasheet (SGT meaning and range)
