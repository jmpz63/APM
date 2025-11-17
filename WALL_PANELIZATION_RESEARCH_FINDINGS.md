# Wall Panelization / Manufacturing – External Research Findings Index

This file consolidates authoritative external references (PDFs and documentation pages) relevant to the wall panel automation initiative: electronics, motion control, mechanical transmission, planning software.

> Canonical location: root-level for quick discovery. Duplicate copies inside submodule-like directories are avoided. If a structured docs tree emerges later, this can migrate with a redirect stub left behind.

## 1. Control Electronics & Drivers
| Topic | Link | Why It Matters |
|-------|------|----------------|
| BIGTREETECH Octopus Max EZ User Manual | https://github.com/bigtreetech/Documents/tree/master/Octopus%20MAX%20EZ | Pinouts, stepper slot CS mapping, voltage domains |
| Octopus Max EZ Schematic | (same folder) | Signal tracing, SPI bus integrity, endstop routing |
| TMC5160A Datasheet | https://www.analog.com/media/en/technical-documentation/data-sheets/tmc5160a.pdf | Register map, current scaling, diagnostics flags |
| SilentStepStick TMC5160 Reference | https://github.com/watterott/SilentStepStick/blob/master/hardware/TMC5160_v12.pdf | Practical layout and decoupling example |
| STM32H723 MCU Datasheet | https://www.st.com/resource/en/datasheet/stm32h723zg.pdf | Electrical limits for timing + power planning |
| STM32H7 Reference Manual (RM0468) | https://www.st.com/resource/en/reference_manual/rm0468-stm32h723-stm32h733-and-stm32h725-stm32h735-advanced-armbased-32-bit-mcus-stmicroelectronics.pdf | In-depth peripherals (SPI, timers, DMA) |

## 2. Actuators (Stepper Motors & Planetary Gearboxes)
Match the joint motor models in the Moveo derivative design.
| Joint | Model | Datasheet | Torque Curve |
|-------|-------|-----------|--------------|
| J1 | 17HS24-0644S | https://www.omc-stepperonline.com/download/17HS24-0644S.pdf | https://www.omc-stepperonline.com/download/17HS24-0644S_Torque_Curve.pdf |
| J2 | 23HS32-4004S | https://www.omc-stepperonline.com/download/23HS32-4004S.pdf | https://www.omc-stepperonline.com/download/23HS32-4004S_Torque_Curve.pdf |
| J3 | 17HS19-1684S-PG14 | https://www.omc-stepperonline.com/download/17HS19-1684S-PG14.pdf | https://www.omc-stepperonline.com/download/17HS19-1684S-PG14_Torque_Curve.pdf |
| J4 | 17HS13-0404S-PG5 | https://www.omc-stepperonline.com/download/17HS13-0404S-PG5.pdf | https://www.omc-stepperonline.com/download/17HS13-0404S-PG5_Torque_Curve.pdf |
| J5 | 14HS17-0504S | https://www.omc-stepperonline.com/download/14HS17-0504S.pdf | https://www.omc-stepperonline.com/download/14HS17-0504S_Torque_Curve.pdf |
| J6 | 8HS15-0604S-PG19 | https://www.omc-stepperonline.com/download/8HS15-0604S-PG19.pdf | https://www.omc-stepperonline.com/download/8HS15-0604S-PG19_Torque_Curve.pdf |

## 3. Motion & Firmware References
| Topic | Link | Notes |
|-------|------|-------|
| Klipper Config Reference | https://www.klipper3d.org/Config_Reference.html | Single source for object parameters |
| Klipper Manual Stepper | https://www.klipper3d.org/Manual-Config.html | Keys for `[manual_stepper]` sections |
| MoveIt 2 Tutorials (ROS 2) | https://moveit.picknik.ai/humble/index.html | Planning pipeline, kinematics adapters |
| ROS 2 Control | https://control.ros.org/ | Hardware interface architecture for multi-joint systems |

## 4. Mechanical & Transmission Components
| Topic | Link | Why |
|-------|------|-----|
| HIWIN Linear Guideway Catalog | https://www.hiwin.com/pdf/Linear_Guideway.pdf | Load ratings, deflection, preload classes |
| KHK Gear General Catalog | https://khkgears.net/new/pdf/en_khk_gear_catalog.pdf | Rack & pinion sizing, module selection |
| KHK Gear Technical Reference | https://khkgears.net/new/pdf/gear_technical_reference.pdf | Backlash, face width, strength calcs |

## 5. Original BCN3D Moveo Context
| Topic | Link | Use |
|-------|------|-----|
| BCN3D Moveo Repository | https://github.com/BCN3D/BCN3D-Moveo | Legacy design decisions & assembly guidance |

## 6. Suggested Local Organization (Optional)
```
/Research/
  wall_panelization/
    electronics/
    actuators/
    mechanics/
    motion_control/
```
Add large PDFs to a `Research/.gitignore` pattern if aggregate >50MB.

## 7. Extraction Targets (Future Automation)
| Parameter | Source | Purpose |
|-----------|--------|---------|
| Motor rated current & torque | Motor datasheets | Drive current limits & acceleration planning |
| Gearbox reduction ratios | Motor gearbox sheets | Effective steps/deg, feed-forward tuning |
| Rail dynamic load rating | HIWIN catalog | Structural FEA & life estimation |
| Backlash & efficiency | Gear technical reference | Path planning error bounds |
| Driver current scaling equations | TMC5160 datasheet | Automated current calibration tool |

## 8. Validation Checklist
- [ ] Verify all links reachable
- [ ] Cache critical driver/motor PDFs offline
- [ ] Record key numbers into a `panelization_hardware.json` (planned)
- [ ] Tie schema v1.0 references to validated hardware ranges

## 9. License & Attribution
All documents are property of their original publishers and provided for internal engineering reference only.

---
Generated: 2025-11-17
Maintainer: (add name/contact)
Change Log:
- 2025-11-17: Initial root index creation.
