# Overhead Conveyor Powder Coating System

## Project Overview
Design and implement an integrated overhead conveyor powder coating process with optimized thermal management for the curing oven. Focus on minimizing heat loss at conveyor openings while maintaining continuous operation and preventing powder blow-off.

## Objectives
- Design overhead monorail conveyor system for continuous powder coating workflow
- Implement efficient thermal barriers (air seals, baffles, vestibules) at oven openings
- Optimize oven airflow (bottom-up pattern) and exhaust control
- Meet NFPA 86 safety requirements while maximizing energy efficiency
- Integrate heater/oven with minimal heat loss at open ends and top slot

## Roles & Cross-links

### APM Hub (This Location)
- **Purpose:** Knowledge base, design specs, thermal engineering analysis, project planning
- **Contents:** Design documents, thermal barrier concepts, airflow calculations, vendor research

### Working Folder
- **Path:** `C:\Users\jmpz6\OneDrive\Powder_Coating\Overhead_Conveyor\` (create if needed)
- **Purpose:** CAD files, detailed mechanical drawings, BOM, wiring diagrams, procurement docs

### Remote Repository
- **GitHub:** TBD (will be created or linked)
- **Purpose:** Version control for design files, specifications, build logs

## Key Technical Challenges

### 1. Heat Retention at Conveyor Openings
- **Problem:** Overhead slot for hanger/product creates continuous heat escape path
- **Solutions:** Air seals (jet streams), unheated vestibules, adjustable baffles
- **Constraints:** Air velocity must not blow powder off freshly coated parts

### 2. Airflow Pattern Optimization
- **Recommended:** Bottom-up airflow (leverages natural buoyancy)
- **Benefit:** Ensures uniform heating without fighting air currents
- **Requirement:** Proper exhaust management to avoid negative pressure

### 3. Exhaust Control
- **Code:** NFPA 86 requires exhaust for VOCs and combustion byproducts
- **Challenge:** Over-exhausting pulls plant air in through openings → heat loss
- **Solution:** VFD-controlled fan calibrated to minimum required rate (4-12 ACH)

## Design Components

### Air Seals (Primary Thermal Barrier)
- High-velocity air curtain across oven opening
- Pulls ambient or recirculated air through narrow slot nozzle
- Creates invisible barrier to block hot air escape
- Critical balance: sufficient velocity to seal, low enough to prevent powder blow-off

### Unheated Vestibule (Air Lock)
- Short enclosed chamber at entrance/exit
- Air seals placed at vestibule ends (not main oven opening)
- Reduces temperature differential between oven and plant
- Allows mixing/cooling of escaped air before reaching environment

### Baffles
- **Internal Fixed Baffles:** Metal plates near openings to redirect hot air downward toward product
- **Flexible Curtains:** High-temp strips (fiberglass/silicone) around entrance/exit
- **Gravity Seals:** Weighted/hinged plates pushed open by hangers, close via gravity

### Insulation
- Minimum 6" mineral wool or equivalent on all surfaces
- Exterior should be cool to touch (if hot, energy is wasted)
- Consider insulated doors/panels for maintenance access

## Next Steps
1. Define conveyor speed, product size, and throughput requirements
2. Calculate required oven chamber dimensions and dwell time for cure
3. Design air seal system: nozzle geometry, air velocity, flow rate
4. Size exhaust fan and VFD for minimum code-compliant ACH
5. Select heater type (gas, electric, infrared) and burner configuration
6. Create mechanical layout with vestibule dimensions
7. Thermal modeling: predict heat loss and energy consumption
8. Vendor research: air curtain units, high-temp curtains, insulation panels

## References
- NFPA 86: Standard for Ovens and Furnaces
- Powder Coating Institute: Curing Oven Design Guidelines
- Knowledge Base: `Engineering/Thermal_Management/` (TBD)

## Status
- **Phase:** Concept / Initial Design
- **Last Updated:** 2025-12-08
- **Owner:** jmpz63

---
*Use APM workflow to update: `notify_new_document` → `ensure_compliance`*
