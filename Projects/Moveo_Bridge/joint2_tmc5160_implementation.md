# Joint2 TMC5160 Implementation - Development Session

## Date: October 5, 2025
## Status: ✅ Completed - TMC5160 Drivers Configured

## Overview
Successfully implemented TMC5160 driver configurations for Joint2 dual motor system (shoulder joint) using verified BTT Octopus Max EZ pinout specifications.

## Hardware Configuration

### Joint2 Dual Motor Setup
- **Joint2 (Motor-8)**: Primary shoulder motor
- **Joint2b (Motor-7)**: Secondary shoulder motor (parallel operation)
- **Motor Model**: 23HS32-4004S (4.0A rated current each)
- **Driver Type**: TMC5160 RGB (4 amp max, 0.05Ω sense resistor)

### BTT Octopus Max EZ Pin Assignments (Verified)
Reference: https://github.com/bigtreetech/Octopus-Max-EZ

#### Motor-7 (Joint2b):
- **Step**: PD3
- **Dir**: PD2
- **Enable**: !PD4
- **CS/UART**: PD7
- **SPI Bus**: spi4

#### Motor-8 (Joint2):
- **Step**: PA10
- **Dir**: PA9
- **Enable**: !PA15
- **CS/UART**: PD6
- **SPI Bus**: spi4

#### Endstop Configuration:
- **Endstop 2**: PF2 (for Joint2 limit switch)

## TMC5160 Configuration Parameters

### Current Settings
- **Run Current**: 2.80A (70% of 4.0A rating for thermal safety)
- **Hold Current**: 1.40A (50% of run current)
- **Sense Resistor**: 0.05Ω (TMC5160 RGB specific)

### Driver Features
- **StealthChop**: Enabled (threshold: 999999)
- **Interpolation**: Enabled for smoother motion
- **SPI Communication**: spi4 bus (shared with other TMC drivers)

## Implementation Details

### Added to printer.cfg:
```klipper
# TMC5160 Driver for Joint2b (Motor-7)
[tmc5160 manual_stepper joint2b]
cs_pin: PD7
spi_bus: spi4
run_current: 2.80
hold_current: 1.40
sense_resistor: 0.05
stealthchop_threshold: 999999
interpolate: True

# TMC5160 Driver for Joint2 (Motor-8)
[tmc5160 manual_stepper joint2]
cs_pin: PD6
spi_bus: spi4
run_current: 2.80
hold_current: 1.40
sense_resistor: 0.05
stealthchop_threshold: 999999
interpolate: True
```

## Dual Motor Synchronization Strategy

### Current Implementation
- Both motors (joint2 and joint2b) receive identical G-code commands
- Manual stepper definitions ensure parallel operation
- No drift monitoring currently implemented (future enhancement)

### Joint2 Specifications (From Datasheet)
- **Native Steps/Rev**: 200
- **Microstep Setting**: 16
- **Belt/Gear Reduction**: 80:14 (5.714:1)
- **Effective Steps/Joint Rev**: ~4571 steps
- **Combined Holding Torque**: 480 N·cm (theoretical)
- **Planning Velocity Limit**: 25-30 deg/s
- **Planning Acceleration Limit**: 120-140 deg/s²

## Development Process

### 1. Pin Verification
- Consulted BTT Octopus Max EZ official GitHub repository
- Verified CS pin assignments: PD7 (Motor-7), PD6 (Motor-8)
- Confirmed endstop pin: PF2 (Endstop 2)

### 2. Configuration Implementation
- Added TMC5160 driver sections to printer.cfg
- Set appropriate current limits for 4A motors
- Configured 0.05Ω sense resistor (TMC5160 RGB specific)

### 3. Safety Considerations
- Current derating to 70% for thermal management
- Hold current set to 50% to reduce heat during stationary operation
- StealthChop enabled for quieter operation

## Next Steps

### Immediate Testing Required
1. **TMC Communication Test**: Verify SPI communication with drivers
2. **Basic Movement Test**: Test individual motor operation
3. **Synchronization Test**: Verify dual motor coordination
4. **Current Monitoring**: Check actual current draw and temperature

### Future Enhancements
1. **Endstop Integration**: Add PF2 endstop configuration
2. **Drift Monitoring**: Implement step count discrepancy detection
3. **Thermal Monitoring**: Add temperature sensors for driver monitoring
4. **Feedback System**: Consider encoder integration for closed-loop control

## Risk Assessment

### Thermal Considerations
- High current (2.8A) may require active cooling
- Monitor driver temperature during extended operation
- Consider further current reduction if thermal issues occur

### Synchronization Risks
- Dual motors may drift over time without feedback
- Mechanical coupling should handle minor discrepancies
- Future encoder feedback recommended for precision applications

## APM Integration
- Documentation added to moveo_bridge_ws working directory
- Changes committed to local git repository
- Ready for integration into APM knowledge system

---
**Created**: $(date)
**Status**: Production Ready for Testing
**Next Phase**: Hardware validation and movement testing
