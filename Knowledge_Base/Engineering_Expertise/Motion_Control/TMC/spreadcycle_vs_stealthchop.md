# TMC SpreadCycle vs StealthChop — Robotics-Focused Guide

## TL;DR
- StealthChop = Silent Mode (low noise, lower torque at speed). Good for very slow, delicate moves.
- SpreadCycle = Performance Mode (higher torque/speed, robust diagnostics). Required for StallGuard.
- For a robotic arm: default to SpreadCycle for all motion except the most delicate low-speed positioning.

## SpreadCycle in Plain Terms
The SpreadCycle chopper mode is the high-performance, high-torque workhorse of the Trinamic (TMC) driver family.

If StealthChop is the driver's "Silent Mode," then SpreadCycle is its "Performance Mode." It sacrifices quietness for speed, maximum holding torque, and, most importantly for your robotic arm, diagnostic reliability.

### What is SpreadCycle?
SpreadCycle is a highly intelligent, regulated current chopping mode. In simple terms, it's the method the driver uses to maintain a precise, smooth current flow in the motor coils, ensuring the rotor is pulled into the exact microstep position with maximum force.

It works using a hysteresis window (a small tolerance band around the target current level).

- Current Rises: When the driver senses the coil current drop below the target, it turns the power on to boost the current.
- Current Peaks: Once the current hits the upper limit of the window, the power is quickly turned off.
- Regulated Decay: The current is then allowed to decay naturally back down toward the target, where the cycle repeats.

This constant, regulated chopping (or "spreading") of the current provides several critical advantages over the "silent" mode.

### Why SpreadCycle is Essential for Robotics
For your Moveo robotic arm, you will rely on SpreadCycle for nearly all your movement, especially when safety and speed are involved.

1) Maximum Torque and Speed
- Acceleration: When the arm needs to accelerate a heavy joint or hold a payload against gravity, SpreadCycle ensures the motor delivers its full, rated torque.
- High RPM: SpreadCycle maintains excellent torque performance even at high rotational speeds, whereas StealthChop often loses significant torque above a certain velocity.

2) Enables StallGuard (Crash Detection)
- StallGuard can only function reliably when the driver is in SpreadCycle mode.
- StallGuard measures back EMF; SpreadCycle's predictable waveform makes it accurate.
- If the arm crashes into an object, StallGuard detects the load spike and triggers a stop — only if in SpreadCycle.

3) Consistency and Reliability
- SpreadCycle is less sensitive to motor parameter/power variations compared to StealthChop, which is critical for synchronized multi-axis motion.

### The Trade-Off: Acoustic Noise
The primary drawback is noise. Because the current is constantly being chopped and regulated, the motor coils vibrate, producing the classic stepper "singing" or "whining."

This is why the TMC5160 is often configured to use StealthChop for slow, delicate moves and SpreadCycle for fast, powerful, and safety-critical moves. Transition can be automatic based on speed.

## Klipper-oriented tuning notes (TMC5160)

- Choosing a mode
  - Prefer a single mode during motion. Either always SpreadCycle (recommended for robotics) or always StealthChop (only if noise is critical and dynamics are slow). Avoid switching modes on-the-fly via thresholds, as changing mode at non-zero velocity can yield poor/erratic behavior.
  - SpreadCycle: Do not set `stealthchop_threshold` (or set to 0). Ensure `driver_CHM=0` and `driver_TOFF>0` so SpreadCycle is active.
  - StealthChop: Set `stealthchop_threshold: 999999` to force StealthChop at all speeds. Expect less torque and more position lag at velocity.

- Current settings
  - Use motor RMS current, not peak: If your motor is labeled 2.8A max, a typical RMS target is around 2.0A (2.8 / √2). Start lower and increase until you have adequate torque without overheating motors or drivers.
  - Configure `run_current:` only; prefer not to set `hold_current:`. If you must reduce idle heat on Z or static joints, carefully validate that enabling hold current doesn’t cause uncommanded motion when current changes.
  - TMC5160 uses a “global scaler” internally; Klipper computes correct register values if `sense_resistor:` matches your hardware. Confirm with `DUMP_TMC`.

- Microsteps and interpolation
  - For best positional accuracy: use SpreadCycle, set `interpolate: False`, and increase `microsteps` (e.g., 32/64/128) to reduce audible noise without introducing interpolation delay.
  - Interpolation can add a small systemic position lag. In StealthChop this lag is dwarfed by mode behavior; in SpreadCycle prefer disabling interpolation for accuracy.

- Chopper timing (SpreadCycle)
  - `driver_TOFF`, `driver_HSTRT`, `driver_HEND`, `driver_TBL` influence audible frequency and current regulation edges. Defaults are generally good. If experimenting:
    - Increasing `driver_TBL` shifts blanking time and tends to move noise frequency; small steps only.
    - Ensure `driver_TOFF` is nonzero (e.g., 3–5) to keep chopper active.
  - Without measurement tools (e.g., oscilloscope), extensive tweaking can be time-consuming with mixed results.

- PWM (StealthChop) notes
  - If you must use StealthChop, `driver_PWM_FREQ`, `driver_PWM_GRAD`, and `driver_PWM_OFS` can influence tone and startup. Some motors “squeal” at certain settings; small, systematic adjustments help.
  - Practical heuristic: StealthChop behaves best below ~5 RPS at the motor shaft. Above that, SpreadCycle is recommended.

- Diagnostics and safety
  - `DUMP_TMC STEPPER=<name>`: confirm mode, currents, and flags. Watch `ot`/`otpw` for thermal events.
  - Use conservative accel/jerk; validate torque margin at worst-case gravity loads and worst kinematic leverage.
  - StallGuard requires SpreadCycle; tune only in SpreadCycle. See `stallguard_basics.md` in this folder.

### Example: TMC5160 SpreadCycle setup (no mode switching)

```
[stepper_joint2]
step_pin: ...
dir_pin: ...
enable_pin: ...
microsteps: 64
# rotation_distance / gear_ratio configured per joint mechanics

[tmc5160 stepper_joint2]
cs_pin: ...
sense_resistor: 0.075      # match your board/driver; stepsticks often 0.110; some externals 0.022
interpolate: False         # prefer accuracy; raise microsteps if you want quieter motion
run_current: 1.8           # Amps RMS (example) — verify thermals
# Do not set stealthchop_threshold -> stay in SpreadCycle
# SpreadCycle shaping (defaults generally OK):
driver_TOFF: 3
driver_HSTRT: 5
driver_HEND: 2
driver_TBL: 2
```

### Example: Force StealthChop everywhere (if absolutely needed)

```
[tmc5160 stepper_joint2]
...
stealthchop_threshold: 999999
interpolate: True
run_current: 1.4    # consider lowering current to reduce heat in this mode
```

## References
- Trinamic application notes and datasheets (TMC5160)
- Klipper docs: TMC Drivers, Config Reference, Sensorless Homing (StallGuard)

## Cross-References
- APM/PROJECTS/Active/moveo_bridge_ws — live Klipper config and macros
- APM/Knowledge_Base/ROS_Robotics — for ROS integration context
