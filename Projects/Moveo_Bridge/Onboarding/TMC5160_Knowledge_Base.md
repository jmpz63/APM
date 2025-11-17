# TMC5160 Driver Knowledge Base

## Overview
The TMC5160 is a high-performance stepper motor driver with integrated motion controller, supporting up to 4.2A RMS current with SPI communication and advanced features like StealthChop, SpreadCycle, and stallGuard.

## Klipper TMC Commands Reference

### Basic Commands
```gcode
DUMP_TMC STEPPER=joint2           # Show all TMC register values
DUMP_TMC                          # Show all TMC drivers
SET_TMC_CURRENT STEPPER=joint2 CURRENT=2.8 HOLDCURRENT=2.0  # Runtime current adjustment
```

### What DUMP_TMC Shows vs Configuration
**DUMP_TMC Output**: Raw TMC register values (hexadecimal)
- `drv_status` - Driver status register
- `gstat` - Global status  
- `ioin` - Input/output pin status
- `lost_steps` - Step loss counter
- `tstep` - Time between steps
- `mscnt` - Microstep counter
- `mscuract` - Actual microstep current

**Configuration Parameters** (what we set in printer.cfg):
- `run_current` - RMS current during movement
- `hold_current` - RMS current when stationary  
- `sense_resistor` - Current sense resistor value
- `stealthchop_threshold` - Velocity threshold for StealthChop mode

## TMC5160 Register Deep Dive

### Current Control Registers
| Register | Purpose | Klipper Equivalent |
|----------|---------|-------------------|
| `IHOLD_IRUN` | Hold/Run current | `run_current`, `hold_current` |
| `TPOWERDOWN` | Power down delay | Automatic in Klipper |
| `TPWMTHRS` | PWM/StealthChop threshold | `stealthchop_threshold` |

### Status Registers (DUMP_TMC output)
| Field | Meaning | Values |
|-------|---------|--------|
| `drv_status` | Driver status | Bit field showing driver state |
| `cs_actual` | Actual current scale | 0-31 (current scaling factor) |
| `stallguard` | StallGuard result | 0-1023 (load measurement) |
| `sg_result` | StallGuard filtered | Filtered stallguard value |

### Driver Status Bits (drv_status register)
- Bit 31: `stst` - Standstill indicator
- Bit 30: `olb` - Open load B
- Bit 29: `ola` - Open load A  
- Bit 28: `s2gb` - Short to ground B
- Bit 27: `s2ga` - Short to ground A
- Bit 26: `s2vsb` - Short to supply B
- Bit 25: `s2vsa` - Short to supply A
- Bit 24: `stealth` - StealthChop mode active

## Current Calculation

### RMS to Peak Current Conversion
TMC5160 uses RMS values, but internally works with peak current:
```
Peak Current = RMS Current × √2 × (32/32) / Rsense
```

For 0.075Ω sense resistor:
- 2.8A RMS = ~3.96A peak
- 2.0A RMS = ~2.83A peak

### Current Scale Factor (CS)
The `cs_actual` value in DUMP_TMC shows the current scaling:
- CS = 31 = 100% of configured current
- CS = 15.5 = 50% of configured current
- Formula: `Actual_Current = (CS/31) × Configured_Current`

## Advanced Features

### StealthChop Mode
- **Purpose**: Silent operation at low speeds
- **Configuration**: `stealthchop_threshold: 999999` (always on)
- **Detection**: `stealth` bit in drv_status
- **Trade-off**: Slightly less torque vs noise reduction

### SpreadCycle Mode  
- **Purpose**: Maximum torque at higher speeds
- **Configuration**: `stealthchop_threshold: 0` (disabled)
- **Detection**: `stealth` bit = 0 in drv_status
- **Benefits**: Higher torque, better heat dissipation

### stallGuard (Collision Detection)
- **Purpose**: Sensorless homing and collision detection
- **Configuration**: 
  ```ini
  driver_SGT: 1                    # Sensitivity (0=most sensitive)
  diag0_pin: ^!PG14               # Stallguard trigger pin
  ```
- **Output**: `sg_result` in DUMP_TMC (0-1023)
- **Note**: Requires SpreadCycle mode (no StealthChop)

## Troubleshooting with DUMP_TMC

### Common Issues and Register Values

#### Motor Not Moving
Check `drv_status` for error bits:
- `s2vsa/s2vsb = 1`: Short to supply (check wiring)
- `s2ga/s2gb = 1`: Short to ground (check wiring)  
- `ola/olb = 1`: Open load (check motor connection)

#### Current Too Low/High
Check `cs_actual`:
- Should be close to 31 for full current
- If much lower, check power supply voltage
- If 0, check TMC communication (SPI)

#### Temperature Issues
Monitor driver temperature:
- TMC5160 has internal temperature monitoring
- Reduce current if overheating occurs
- Ensure adequate cooling for high currents (>2A)

## Optimal Settings for BCN3D Moveo

### Joint 2 Shoulder Motors (4.0A rated)
```ini
[tmc5160 manual_stepper joint2]
run_current: 2.8                 # 70% of motor rating
hold_current: 2.0                # 50% of motor rating  
sense_resistor: 0.075            # Standard BTT Octopus value
stealthchop_threshold: 999999    # Always StealthChop for silence
interpolate: True                # Enable 256 microstep interpolation
```

### Performance vs Noise Trade-offs
| Setting | Torque | Noise | Heat | Recommendation |
|---------|--------|-------|------|----------------|
| StealthChop | Medium | Low | Medium | ✅ Recommended |
| SpreadCycle | High | High | Low | Heavy loads only |
| 2.8A Current | High | Medium | High | ✅ Good balance |
| 4.0A Current | Maximum | Medium | Very High | Only if needed |

## Monitoring and Maintenance

### Regular Checks
```gcode
DUMP_TMC STEPPER=joint2          # Check for error flags
DUMP_TMC STEPPER=joint2b         # Verify both motors healthy
```

### Key Values to Monitor
1. **Error flags**: All should be 0 in normal operation
2. **cs_actual**: Should be ~31 for full current output
3. **Temperature**: Driver should not overheat
4. **stallguard**: Should change with load (if enabled)

### Performance Tuning
- Increase current for more torque (up to motor rating)
- Enable stallguard for collision detection
- Use SpreadCycle for maximum performance
- Monitor temperature for sustained high-current operation

---

## References
- [TMC5160 Datasheet](https://www.trinamic.com/fileadmin/assets/Products/ICs/TMC5160A_Datasheet_Rev1.14.pdf)
- [Klipper TMC Documentation](https://www.klipper3d.org/TMC_Drivers.html)
- [BTT Octopus Max EZ Pinout](https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-Max-EZ)