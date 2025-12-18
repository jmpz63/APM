# Rotatable Worktable - Mechanical Specification

## Goals
- Rigid, safe rotation between 0 deg and 180 deg (optional 90 deg)
- Positive mechanical locks; sensor-confirmed index
- Garage-buildable with common steel, bearings, and pneumatics

## Envelope & Deck
- Work area: 4x12 to 5x12 ft
- Deck: torsion box (plywood skins, internal ribs), MDF sacrificial top, 96 mm fixture grid
- Perimeter: steel 2x2x0.120 in tube; mid crossmembers at ~24 in spacing

## Trunnions & Bearings
- End plates: 3/8 in steel with precision bore for trunnion shaft
- Trunnion shaft: 1.5–2.0 in steel shaft; welded/bushed into deck frame
- Bearings: bronze bushings (budget) or tapered roller bearings (rigid)

## Indexing & Locks
- Hard stops at 0/180 (and optional 90) via welded tabs
- Spring-loaded lock pin engaging laser-cut index plate; pin sensor confirms lock
- Rotation brake (mechanical or pneumatic) to prevent free spin

## Actuation
- Manual rotation with assisted handles or geared reduction; motorization optional
- Safety: rotation enabled only when gantry parked and clamps released

## Sensing
- Proximity sensors for index positions; dual sensors for redundancy
- Optional encoder for rotation angle if variable angles are needed

## Integration Interfaces
- Clamp rails and pop-up stops mounting points
- Cable management: fixed umbilicals with slack loops; rotate-safe
- Park zones: clearances defined so gantry “home” is outside rotation arc

## Strength & Torque
- Design torque: >= 3x static torque of heaviest panel about trunnion axis
- Example calc: T = W * g * (offset to trunnion); choose shaft/bearing to exceed T with margin

## Finish & Ergonomics
- Height: 34–36 in
- Edge guards; rounded corners
- Non-slip foot pads; vibration-isolated feet

## BOM (indicative)
- Steel tube (2x2x0.120), plate (3/8 in), shaft stock (1.5–2.0 in)
- Bearings/bushings, lock pins, springs, proximity sensors
- Pneumatic valves/regulators, hoses (if using brake)
- Fasteners, paint, cabling
