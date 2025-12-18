# Hybrid Rotatable Wall Panel Cell - Specification

## Overview
- Stationary rotatable table under an XY gantry nailer
- Robot places lumber (studs/plates) and sheathing
- Table locks at 0 deg and 180 deg before gantry motion; optional 90 deg index
- Safety PLC governs clamps, rotation, nailer air, and interlocks

## Process Sequence
1. Pop-up stops raise; robot places plates and studs against squaring fences
2. Zone clamps engage; clamp sensors confirm "green"
3. XY gantry nails Face A per recipe
4. Gantry parks in safe zone; clamps release; rotation lock disengages
5. Rotate table to 180 deg; locks re-engage; clamps on; nail Face B (or staple sheathing)
6. QA snapshot; label; unload via roller stands

## Table Specs (summary)
- Deck: 4x12-5x12 ft torsion box; 96 mm grid; MDF sacrificial top
- Frame: Steel perimeter (2x2x0.120 in) with cross members; vibration-isolated feet
- Trunnions: End plates with precision bores; bronze bushings or tapered roller bearings
- Locking: Spring-loaded pin + hard stops at 0/180 deg; optional 90 deg detents
- Index sensing: Proximity sensors for lock confirmation; encoder optional

## XY Gantry Specs
- Envelope: Cover table with +50 mm margins; park "home" clear of rotation arc
- Drive: Belt or rack-and-pinion; ~1-2 mm repeatability (framing tolerance)
- Tool: Framing nailer on compliant mount, guarded nose; 24 VDC solenoid for air valve
- Air: Dedicated regulator and accumulator; air off outside safe zones

## Robot Specs (placement)
- Payload: >=10 kg with vacuum/mechanical gripper; compliance springs
- End-effectors:
  - Lumber gripper with soft pads or vacuum cups and beam spreader
  - Sheathing vacuum plate with check valves
- Sensors: Thickness probe or camera for placement verification (optional)

## Clamping & Squaring
- Edge squaring rams; pop-up stops with reed switches
- Zoned pneumatic hold-downs; clamp sensors to confirm engagement
- Clamp map indexed by recipe "zones" to minimize cycle time

## Controls & Safety
- Safety PLC (Pilz/SICK), dual-channel E-stop chain: robots, gantry drives, nail valve, clamps
- Light curtains / interlocked gates; Safe speed monitoring for teach modes
- Interlocks:
  - Gantry enable only when rotation lock = engaged and clamps = on
  - Rotation enable only when gantry = parked and clamps = off
  - Tool air auto-dump on E-stop or clamp faults

## I/O List (example)
- Inputs: rotation lock prox (x2), gantry park prox, clamp sensors (zones), stop up/down sensors, light curtain status
- Outputs: clamp valves (zones), pop-up stops, rotation brake/lock, nailer air solenoid, beacons

## Recipe & Coordinate Frames
- Frame: Table origin at lower-left; Face A coordinates; Face B = 180 deg transform
- JSON recipe includes studs (x,y,length,type), nail patterns (arrays of (x,y)), clamp zones
- Rotation applies transform: A -> B: (x,y) -> (W-x, H-y) with tolerance offsets

## Cycle Targets (baseline)
- Placement: 2-4 min per panel (robot)
- Nailing: 2-6 min per face (gantry)
- Rotation: <30 s including lock/unlock
- Total: 6-12 min/panel depending on density

## Phased Implementation
- Phase 0: Table + manual nailing (templates)
- Phase 1: Add robot for placement; PLC clamps; basic recipes
- Phase 2: Add gantry nailer; integrate interlocks; QA snapshot
- Phase 3: Add sheathing handling; labeling; database of recipes

## BOM (indicative)
- Steel frame + torsion box materials
- Trunnion bearings/bushings; lock pins; proximity sensors
- Pneumatic valves/regs/clamps; pop-up stops; hoses
- Gantry rails/drives; motor drivers; controller
- Safety PLC; light curtains; beacons; wiring
