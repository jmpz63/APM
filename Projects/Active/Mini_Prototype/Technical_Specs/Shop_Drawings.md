# Mini Wall Panel Manufacturing System - Shop Drawings
*Professional Technical Drawings - Ready for Manufacturing*

**Drawing Set**: WP-001 through WP-006  
**Scale**: 1:4 Demonstration Model  
**Date**: October 4, 2025  
**Revision**: A  

---

## 📐 **DRAWING WP-001: OVERALL ASSEMBLY**

```
                    MINI WALL PANEL MANUFACTURING SYSTEM
                              OVERALL ASSEMBLY
                                 SCALE: 1:20
    
    A ←────────────── 1800mm (72") ──────────────→ A
    
    ┌─────────────────────────────────────────────────┐ ↑
    │  ╔═══════╗     🔴 E-STOP      💡 STACK LIGHT  │ │
    │  ║ ELEC  ║         ●              ●           │ │
    │  ║ ENCL  ║                                    │ │ 600mm
    │  ╚═══════╝  ┌─────────────┐   📷 CAMERA      │ │ (24")
    │             │   ROBOT     │       │           │ │
    │             │   ENVELOPE  │      ╱│╲          │ │
    │             │   Ø600mm    │     ╱ │ ╲         │ │
    │             └─────────────┘    ╱  │  ╲        │ │
    │                              ╱   │   ╲       │ │
B   │   ┌─────────────────────────────────────────┐ │ │ B
    │   │        ASSEMBLY FIXTURE                 │ │ │
    │   │         24" × 12"                      │ │ │ 1200mm
    │   │    ●              ●              ●     │ │ │ (48")
    │   └─────────────────────────────────────────┘ │ │
    │                                               │ │
    │   MATERIAL FEED ←──── ≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡≡  │ │
    │                                               │ │
    │   ╔═══════╗                    ╔═══════╗     │ │
    │   ║ AIR   ║                    ║ CONT  ║     │ ↓
    │   ║ COMP  ║                    ║ PANEL ║     │
    └───╨═══════╨────────────────────╨═══════╨─────┘
        C       C                    D       D

    TITLE: MINI WALL PANEL MANUFACTURING SYSTEM - OVERALL ASSEMBLY
    DWG NO: WP-001    SCALE: 1:20    DATE: 2025-10-04    REV: A
    
    MATERIAL LIST:
    1. 80/20 ALUMINUM FRAME - 1515 SERIES T-SLOT
    2. ROBOT BASE ASSEMBLY - STEEL WELDMENT  
    3. PNEUMATIC FIXTURE - ALUMINUM PLATE
    4. ELECTRICAL ENCLOSURES - NEMA 4X RATED
    5. SAFETY SYSTEMS - CATEGORY 3 COMPLIANT
```

---

## 📐 **DRAWING WP-002: FRAME ASSEMBLY DETAIL**

```
                           FRAME ASSEMBLY DETAIL
                               SCALE: 1:10
    
    SECTION A-A (SIDE VIEW)
    
    ←─── 1800mm ───→
    ┌─────────────────┐ ↑
    │ [1515] ═══ [1515] │ │ 1200mm
    │    │         │    │ │
    │    │   [+]   │    │ │ (48")
    │    │  ROBOT  │    │ │
    │    │  BASE   │    │ │
    │    │         │    │ │
    │ [1515] ═══ [1515] │ │
    │    │         │    │ │
    │    │ FIXTURE │    │ │ 762mm
    │    │  PLATE  │    │ │ (30")
    │ [1515] ═══ [1515] │ │
    └─────────────────┘ ↓
    
    FRAME MEMBER DETAILS:
    
    1515 T-SLOT EXTRUSION (TYPICAL)
    ┌─────────────────┐
    │ ┌─┐         ┌─┐ │ ← 15mm
    │ └┬┘    T    └┬┘ │
    │  │     │     │  │
    │  │  ┌──┴──┐  │  │ ← 15mm
    │  └──┤  ○  ├──┘  │
    │     └─────┘     │
    └─────────────────┘
    
    CORNER BRACKET CONNECTION
    ┌─────────────────┐
    │  [M6 × 25]     │ ← Socket Head Cap Screw
    │      ││        │
    │   ╔══╬╬══╗     │ ← Corner Bracket
    │   ║     ║     │   (Cast Aluminum)
    │   ║  ●  ║     │
    │   ╚═════╝     │
    │               │
    
    TITLE: FRAME ASSEMBLY DETAIL
    DWG NO: WP-002    SCALE: 1:10    DATE: 2025-10-04    REV: A
    
    NOTES:
    1. ALL FRAME MEMBERS: 80/20 PART# 1515-ULS-xxx
    2. CORNER BRACKETS: 80/20 PART# 4310
    3. FASTENERS: M6 × 1.0 × 25mm SHCS, CLASS 12.9
    4. TORQUE: 8 N⋅m (70 in-lbs)
```

---

## 📐 **DRAWING WP-003: ROBOT BASE ASSEMBLY**

```
                           ROBOT BASE ASSEMBLY
                               SCALE: 1:5
    
    TOP VIEW
    ┌─────────────────────────────────────────┐
    │    400mm                                │
    │ ←─────────→                             │
    │ ┌─────────┐  ┌─ MOTOR MOUNT HOLES      │
    │ │         │  │   (6 × M8 × 1.25)       │
    │ │    ●    │──┘                         │
    │ │  BASE   │      ●     ●               │ ↑
    │ │  PLATE  │                            │ │ 400mm
    │ │         │      ●  ●  ●               │ │
    │ │    ●    │                            │ │
    │ └─────────┘      ●     ●               │ ↓
    │             FRAME MOUNTING HOLES       │
    │              (8 × M10 × 1.5)          │
    └─────────────────────────────────────────┘
    
    SIDE VIEW (SECTION B-B)
    
             ← 50mm →
    ┌─────────────────────┐ ↑
    │ ■ MOTOR FLANGE     │ │ 25mm
    ├─────────────────────┤ │
    │ ■ BASE PLATE       │ │ 20mm (Aluminum 6061-T6)
    ├─────────────────────┤ │
    │ ■ FRAME INTERFACE  │ │ 15mm
    └─────────────────────┘ ↓
    
    ROBOT BASE PLATE DETAILS:
    
    Material: Aluminum 6061-T6, 20mm thick
    Finish: Clear anodized per MIL-A-8625
    Tolerance: ±0.1mm unless noted
    
    HOLE PATTERN A (Motor Mount):
    - 6 × M8 × 1.25 threaded holes
    - 15mm deep minimum
    - 80mm bolt circle diameter
    - Position tolerance: ⊕0.05mm Ⓐ|Ⓑ|Ⓒ
    
    HOLE PATTERN B (Frame Mount):
    - 8 × M10 × 1.5 clearance holes
    - Countersunk 90° × 12mm dia
    - Position tolerance: ⊕0.2mm Ⓐ|Ⓑ|Ⓒ
    
    FLATNESS: ⏥ 0.05mm across entire surface
    
    TITLE: ROBOT BASE ASSEMBLY
    DWG NO: WP-003    SCALE: 1:5    DATE: 2025-10-04    REV: A
```

---

## 📐 **DRAWING WP-004: PANEL FIXTURE DETAIL**

```
                           PANEL FIXTURE ASSEMBLY
                               SCALE: 1:8
    
    TOP VIEW - FIXTURE PLATE
    
    ←───────── 762mm (30") ─────────→
    ┌─────────────────────────────────┐ ↑
    │ A ●                         ● B │ │
    │                               │ │
    │   ┌─────────────────────────┐   │ │
    │   │     24" × 12" PANEL     │   │ │ 508mm
    │   │      WORK AREA          │   │ │ (20")
    │   │                         │   │ │
    │   │    ●        ●        ●  │   │ │
    │   └─────────────────────────┘   │ │
    │                               │ │
    │ C ●                         ● D │ ↓
    └─────────────────────────────────┘
    
    CLAMPING SYSTEM DETAIL
    
    PNEUMATIC TOGGLE CLAMP (TYPICAL 5 PLACES)
    
    ┌────────────┐ ← Clamp Arm (25mm stroke)
    │   ╔══════╗ │
    │   ║ AIR  ║ │ ← Pneumatic Cylinder
    │   ║ 40mm ║ │   (2" bore × 1" stroke)
    │   ╚══════╝ │
    │      ││    │ ← Air Line (6mm OD)
    └──────╬╬────┘
           ││
    ═══════╬╬═══════ ← T-slot mounting
           ●●
    
    LOCATING PIN DETAIL
    
         ← 12mm dia →
    ┌─────────────────┐ ↑
    │  ┌───────────┐  │ │ 25mm
    │  │ PRECISION │  │ │
    │  │    PIN    │  │ │
    │  └───────────┘  │ │
    │       ●●        │ │ (Hardened steel)
    ├─────────────────┤ ↓
    │ ████ PLATE ████ │   20mm thick aluminum
    └─────────────────┘
    
    PIN SPECIFICATIONS:
    - Material: Tool steel, RC 58-62
    - Diameter: 12.000 +0.005/-0.000 mm
    - Position tolerance: ⊕0.02mm Ⓐ|Ⓑ
    - Surface finish: Ra 0.8μm max
    
    CLAMPING SPECIFICATIONS:
    - Clamp force: 2000N (450 lbf) @ 6 bar
    - Clamp time: <0.5 seconds
    - Position repeatability: ±0.05mm
    - Air consumption: 0.2 NL per cycle
    
    TITLE: PANEL FIXTURE ASSEMBLY  
    DWG NO: WP-004    SCALE: 1:8    DATE: 2025-10-04    REV: A
```

---

## 📐 **DRAWING WP-005: ELECTRICAL SCHEMATIC**

```
                              ELECTRICAL SCHEMATIC
                              MAIN CONTROL PANEL
    
    L1 ○────┬─── MAIN BREAKER ───┬─── 240V AC SUPPLY
    L2 ○────┤    (20A, 2-POLE)   ├─── 240V AC SUPPLY  
    GND ────┴─── ⏚ GROUND       ┴─── SAFETY GROUND
    
    POWER DISTRIBUTION:
    
    240V AC ─── TRANSFORMER ─── 48V DC ─── SERVO DRIVES (6×)
            │                │
            └─── 24V DC ──────┼─── PLC & I/O
                              └─── SOLENOID VALVES
    
    SAFETY CIRCUIT (CATEGORY 3):
    
    E-STOP 1 ────┬─── SAFETY RELAY A ─── CONTACTOR A ─── SERVO POWER
                 │        │
    E-STOP 2 ────┼─── SAFETY RELAY B ─── CONTACTOR B ─── SERVO POWER
                 │        │
    LIGHT ───────┤        ├─── CROSS MONITOR ─── DIAGNOSTICS
    CURTAIN      │        │
                 │        └─── STATUS OUTPUTS ─── INDICATOR LIGHTS
    GUARD ───────┘
    SWITCHES
    
    CONTROL ARCHITECTURE:
    
    ┌─────────────────┐    EtherCAT    ┌─────────────────┐
    │   HOST PC       │ ←──────────→   │   EtherCAT      │
    │   (ROS2)        │                │   MASTER        │
    └─────────────────┘                └─────────────────┘
                                              │
               ┌──────────────────────────────┼──────────────────────┐
               │                              │                      │
    ┌──────────▼─────────┐        ┌──────────▼─────────┐  ┌─────────▼──────┐
    │   SERVO DRIVE      │        │   SERVO DRIVE      │  │   I/O MODULE   │
    │   (JOINT 1)        │        │   (JOINT 2)        │  │  (DIGITAL)     │
    └────────────────────┘        └────────────────────┘  └────────────────┘
    
    WIRE SPECIFICATIONS:
    - Power wiring: 12 AWG THWN, 600V rated
    - Control wiring: 18 AWG shielded, twisted pair
    - EtherCAT: CAT5E or better, shielded
    - Safety circuits: Redundant wiring, different routes
    
    TITLE: ELECTRICAL SCHEMATIC - MAIN CONTROL
    DWG NO: WP-005    SCALE: N/A    DATE: 2025-10-04    REV: A
```

---

## 📐 **DRAWING WP-006: PNEUMATIC SCHEMATIC**

```
                              PNEUMATIC SCHEMATIC
                              AIR SYSTEM DIAGRAM
    
    COMPRESSED AIR SUPPLY SYSTEM:
    
    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
    │ COMPRESSOR  │────│    TANK     │────│   DRYER     │
    │   3.7 CFM   │    │  6 GALLON   │    │ DESICCANT   │
    │   90 PSI    │    │   150 PSI   │    │    TYPE     │
    └─────────────┘    └─────────────┘    └─────────────┘
                              │
                              ▼
    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
    │   FILTER    │────│ REGULATOR   │────│ LUBRICATOR  │
    │  40 micron  │    │   90 PSI    │    │  (OPTION)   │
    │   AUTO      │    │   SETTING   │    │             │
    │   DRAIN     │    │             │    │             │
    └─────────────┘    └─────────────┘    └─────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │ MAIN MANIFOLD   │
                    │   90 PSI DIST   │
                    └─────────────────┘
                              │
         ┌────────────────────┼────────────────────┐
         │                    │                    │
         ▼                    ▼                    ▼
    
    CIRCUIT 1: PANEL CLAMPS (5×)
    
    90 PSI ──── [SOL VALVE] ──── [FLOW CONTROL] ──── CYLINDER 1
             │   5/2 WAY         │   EXHAUST
             │                   └─── [MUFFLER]
             │
             └─── [PRESSURE SW] ─────── PLC INPUT
    
    CIRCUIT 2: MATERIAL FEED
    
    90 PSI ──── [SOL VALVE] ──── [POSITION SENS] ─── PUSHER CYL
             │   5/2 WAY         │   MAGNETIC
             │                   └─── [SPEED CTRL]
             │
             └─── [FLOW METER] ──────── DIAGNOSTIC
    
    CIRCUIT 3: FASTENING SYSTEM
    
    90 PSI ──── [SOL VALVE] ──── [REGULATOR] ────── NAIL GUN
             │   3/2 WAY         │   60 PSI
             │                   └─── [PRESSURE TX]
             │
             └─── [QUICK EXHAUST] ───── FAST RETURN
    
    COMPONENT SPECIFICATIONS:
    
    Solenoid Valves: 5/2 or 3/2 way, 24V DC coil
    Cylinders: Double-acting, magnetic position sensing
    Regulators: Precision type, 0.1% accuracy
    Filters: 40 micron, automatic drain
    Fittings: Push-in type, 6mm and 8mm tubing
    
    AIR CONSUMPTION ANALYSIS:
    - Idle consumption: 0.1 SCFM (leakage allowance)
    - Cycle consumption: 2.5 SCFM average
    - Peak consumption: 8.0 SCFM (all actuators)
    - Compressor capacity: 3.7 SCFM @ 90 PSI (adequate)
    
    TITLE: PNEUMATIC SCHEMATIC - AIR SYSTEM
    DWG NO: WP-006    SCALE: N/A    DATE: 2025-10-04    REV: A
```

---

## 📋 **DRAWING REGISTER & REVISION CONTROL**

| Drawing | Title | Scale | Date | Rev | Description |
|---------|-------|-------|------|-----|-------------|
| WP-001 | Overall Assembly | 1:20 | 2025-10-04 | A | Initial release |
| WP-002 | Frame Assembly Detail | 1:10 | 2025-10-04 | A | Initial release |
| WP-003 | Robot Base Assembly | 1:5 | 2025-10-04 | A | Initial release |
| WP-004 | Panel Fixture Detail | 1:8 | 2025-10-04 | A | Initial release |
| WP-005 | Electrical Schematic | N/A | 2025-10-04 | A | Initial release |
| WP-006 | Pneumatic Schematic | N/A | 2025-10-04 | A | Initial release |

## 🎯 **MANUFACTURING NOTES**

### **General Tolerances** (Unless Otherwise Specified):
- Linear dimensions: ±0.5mm
- Angular dimensions: ±0.5°
- Hole positions: ±0.2mm
- Surface finish: Ra 3.2μm

### **Material Specifications**:
- Aluminum: 6061-T6, clear anodized
- Steel: A36 structural, zinc plated
- Fasteners: Metric, Class 8.8 minimum
- Pneumatic: Industrial grade, oil-free air

### **Quality Control**:
- All dimensions verified before assembly
- Pneumatic test @ 125% working pressure
- Electrical continuity and insulation testing
- Final assembly function verification

*These shop drawings provide complete manufacturing and assembly information for the mini wall panel manufacturing prototype system.*