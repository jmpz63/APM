# TMC5160 Tuning Cheat Sheet (Klipper)

Goal: Stable, accurate motion with predictable acoustics and safe thermals.

## 1) Wiring sanity and comms
- SPI lines correct; CS per driver; shared SPI devices fully configured or CS held high.
- DIAG line wired for StallGuard (optional but recommended). Use `diag1_pin` or `diag0_pin`.
- Motor power present; run `DUMP_TMC STEPPER=<name>` — no SPI errors; registers make sense.

## 2) Baseline config (SpreadCycle, accurate)

```
[tmc5160 stepper_*]
cs_pin: ...
sense_resistor: 0.075       # match hardware; common stepsticks 0.110; BTT Plus ext. ~0.022
interpolate: False          # prefer accuracy
run_current: <amps RMS>     # start ~ 0.6–0.8 × motor RMS capability; raise if needed
# Do NOT set stealthchop_threshold → SpreadCycle only
driver_TOFF: 3
driver_HSTRT: 5
driver_HEND: 2
driver_TBL: 2
```

Tips
- Increase microsteps (32/64/128) instead of enabling `interpolate` for quieter motion with accuracy.
- Avoid `hold_current` initially; add later only for Z/static axes after validation.

## 3) Current and thermals
- Motor label current is often peak; RMS ≈ peak / √2.
- Raise `run_current` until torque margin is sufficient without ot/otpw.
- Add airflow to drivers and motors for higher currents.

## 4) Noise shaping (optional)
- SpreadCycle tone tweaks: small changes to `driver_TBL` and `driver_TOFF` can shift audible frequency.
- StealthChop (only if required): set `stealthchop_threshold: 999999`. Expect lower torque and position lag at speed. Adjust `driver_PWM_FREQ`, `driver_PWM_OFS`, `driver_PWM_GRAD` minimally.

## 5) StallGuard (sensorless) set-up
- Use SpreadCycle only.
- Map virtual endstop: `endstop_pin: tmc5160_stepper_*:virtual_endstop` and `homing_retract_dist: 0`.
- Tune SGT using Klipper flow; final `driver_SGT` ~ min + (max − min)/3.

## 6) Validation checklist
- Slow jogs: smooth, no harsh ticking.
- Long moves: no missed steps, expected arrival position, acceptable tone.
- Worst-case load/lever: still no skipping at planned accel/vel.
- DUMP_TMC: no `ot`, `uv_cp`, or resets under workload.

## 7) Common pitfalls
- Wrong `sense_resistor` value → completely wrong current scaling.
- Mode switching via `stealthchop_threshold` → erratic behavior at non-zero velocity.
- Using `hold_current` on axes under load → uncommanded motion when current changes.
- Shared SPI without configuring all devices → random SPI write failures.

## 8) Quick field commands
- Check fields: `DUMP_TMC STEPPER=stepper_*`
- Change low-level field: `SET_TMC_FIELD STEPPER=stepper_* FIELD=HEND VALUE=3`
- Temporary current: `SET_TMC_CURRENT STEPPER=stepper_* CURRENT=1.4`

## References
- Klipper: TMC Drivers, Config Reference, Sensorless Homing
- Trinamic TMC5160 datasheet (SpreadCycle/StealthChop/SGT)
