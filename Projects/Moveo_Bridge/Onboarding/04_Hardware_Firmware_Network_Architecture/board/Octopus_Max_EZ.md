# BigTreeTech Octopus Max EZ Board Notes

MCU: STM32H723ZET6 (Cortex-M7)

Config reference: `~/printer_data/config/printer.cfg`

## Key References
- Vendor Repo: https://github.com/bigtreetech/Octopus-Max-EZ
- Datasheet (MCU): STMicroelectronics STM32H723xE
- Klipper Sample Config (search in klipper repo for h723 / octopus max): `generic-bigtreetech-octopus-max-ez-h723.cfg` (name may vary)
- Board Manual PDF (local once added): `board/datasheets/BIGTREETECH_Octopus_MAX_EZ_V1.0_User_Manual.pdf`

## Core Specs
| Item | Value |
|------|-------|
| MCU | STM32H723ZET6 (M7, 550 MHz max; typical Klipper freq lower) |
| Flash | 512 KB |
| RAM | 564 KB SRAM (domain partitioned) |
| Voltage Domains | 3.3V logic; 5V & VIN rails for peripherals |
| USB | Native High-speed (FS via internal PHY) |

## Fan / MOSFET Outputs (Populate After Verification)
| Label (Silkscreen) | Function | MCU Pin | Notes |
|--------------------|----------|---------|-------|
| FAN0               | PWM MOSFET | PA6    | Configured as `[fan]` (generic) – tested target |
| FAN1               | PWM MOSFET | (TBD)  |  |
| FAN2               | PWM MOSFET | (TBD)  |  |
| FAN3               | PWM MOSFET | (TBD)  |  |

## Fan0 Test Procedure
1. Ensure 24V (or board VIN) is present; fan plugged into FAN0 header observing polarity.
2. In console (Mainsail/Fluidd):
   - `M106 S255` (turn full on) OR `SET_FAN_SPEED FAN=fan SPEED=1` (if named `[fan]`).
   - `M106 S0` (turn off).
3. Observe rotation / measure voltage at FAN0 output (~VIN).
4. If no motion:
   - Confirm `pin: PA6` present in `[fan]` section.
   - Check jumper selecting fan voltage (if applicable on board revision).
   - Use multimeter to verify voltage when on.
5. Optional HTTP test:
```bash
curl -X POST -H 'Content-Type: application/json' \
  -d '{"script":"M106 S255"}' http://<host>:7125/printer/gcode/script
```
Then off:
```bash
curl -X POST -H 'Content-Type: application/json' \
  -d '{"script":"M106 S0"}' http://<host>:7125/printer/gcode/script
```

## Recommended Klipper Fan Config Example
```ini
[fan_generic fan1]
pin: FAN1  # Use alias from [board_pins] if available
max_power: 1.0
shutdown_speed: 0
kick_start_time: 0.3
```
If raw pin required (example only – replace):
```ini
[fan_generic fan1]
pin: PE5
```

## Board Pins Alias Strategy
If sample config contains:
```ini
[board_pins octopus_max_ez]
aliases:
  FAN0=PF7
  FAN1=PE5
  FAN2=PE6
  FAN3=PG0
```
Then always prefer the alias (easier future migration).

## Debugging Fans
1. Confirm VIN present (measure with multimeter).
2. Check jumper selecting fan voltage (if board supports 12V/24V domains).
3. Issue `SET_FAN_SPEED FAN=fan1 SPEED=1`. If no spin, test voltage at header.
4. Try direct pin with `output_pin` test section if alias fails.

## Firmware Build Target
When compiling Klipper MCU firmware choose `STM32H723` and correct clock/ref (usually 12 MHz crystal if present – verify). Use CAN/USB selection per your wiring.

## Action Items
- [ ] Pull exact fan pin mapping from official schematic.
- [ ] Populate MOSFET table.
- [ ] Add any jumpers or power rail configuration notes.
- [ ] Note maximum recommended fan current (per MOSFET spec).

---
Last update: 2025-10-01 initial board note created.

## Current Integration Notes (2025-10-01)
- FAN0 configured with delayed_gcode spin-up (full 255 → settle 166 ~65%).
- Conditional joint-activity fan macros removed for simplicity.
- Base joint positive endstop characterization in progress using staged macros (see Project Manual Section 0.4.12).
