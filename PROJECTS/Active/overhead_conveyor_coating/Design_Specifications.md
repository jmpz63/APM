# Design Specifications: Overhead Conveyor Powder Coating Oven

## Air Seal (Air Curtain) Specifications

### Purpose
Create high-velocity air barrier across oven openings to minimize heat loss while preventing powder blow-off on freshly coated parts.

### Key Parameters

#### Air Velocity Range
- **Minimum Effective:** 1000 FPM (feet per minute)
- **Typical Range:** 1500–2500 FPM
- **Maximum (Powder Safety):** 3000 FPM (above this risks disturbing uncured powder)
- **Adjustment:** VFD-controlled blower for commissioning and seasonal tuning

#### Nozzle Design
- **Slot Width:** 0.5"–1.5" (narrow slot = higher velocity at lower CFM)
- **Span:** Full width of conveyor opening + 6" overlap each side
- **Angle:** Directed 10–20° inward (toward oven interior) for overhead applications
- **Placement:** Top and bottom of opening (dual-curtain recommended for tall openings >24")

#### Air Source
- **Option A:** Recirculated oven air (most energy-efficient; requires heat-resistant blower)
- **Option B:** Ambient plant air (simpler, but introduces cooler air; net heat loss)
- **Recommendation:** Recirculated air for continuous high-temp ovens (>350°F)

#### Flow Rate Calculation
```
CFM = (Opening Height [ft] × Opening Width [ft] × Desired Velocity [FPM]) / Slot Width [in] × 12
```
Example: 
- Opening: 24" H × 36" W (2 ft × 3 ft)
- Velocity: 2000 FPM
- Slot: 1"
- CFM = (2 × 3 × 2000) / (1 × 12) = 1000 CFM per curtain

#### Blower Sizing
- **Static Pressure:** 2"–4" WC (water column) typical for slot nozzle
- **Motor:** 1–3 HP per air curtain (depends on CFM and static pressure)
- **Control:** VFD mandatory for tuning and energy savings

---

## Unheated Vestibule Specifications

### Dimensions
- **Length:** 3–6 ft (longer = better thermal buffer, but adds cost/footprint)
- **Height/Width:** Match conveyor clearance + 12" margin for access
- **Ceiling Clearance:** Allow for product + hanger + 6" minimum

### Construction
- **Walls:** Insulated panels, R-15 minimum (half of main oven insulation)
- **Access:** Personnel door on side for maintenance (insulated, gasketed)
- **Lighting:** LED fixtures rated for ambient temp (vestibule is unheated, but may reach 100–150°F via convection)

### Air Seal Placement
- Install air curtain units at the **outer openings** of vestibule (plant-side)
- Allows vestibule to act as mixing/buffer zone between oven and plant

### Optional: Internal Circulation
- Low-velocity fan (100–300 CFM) to prevent stratification
- Helps maintain uniform temperature in vestibule (improves air seal effectiveness)

---

## Insulation Requirements

### Main Oven Chamber
- **Walls:** 6" mineral wool or ceramic fiber (R-30+)
- **Ceiling:** 6" mineral wool (R-30+)
- **Floor:** 4" high-density board insulation under conveyor path (if applicable)
- **Exterior Surface Temp:** <120°F at 400°F oven setpoint (safety and efficiency)

### Vestibule
- **Walls/Ceiling:** 3"–4" mineral wool or equivalent (R-15)
- **Purpose:** Reduce radiant heat transfer to plant; not a primary heated zone

### Access Doors/Panels
- **Insulated swing doors** for burner access, maintenance
- **Gaskets:** High-temp silicone or ceramic fiber rope seal
- **Latches:** Compression-style to ensure tight seal when closed

---

## Exhaust System Specifications

### Code Compliance
- **NFPA 86:** Requires exhaust for VOC removal and combustion air management
- **Air Changes per Hour (ACH):** 4–12 ACH typical for powder cure (lower end if using recirculated air with filtration)

### Exhaust Fan Sizing
```
CFM_exhaust = (Oven Volume [ft³] × ACH) / 60
```
Example:
- Oven: 10 ft L × 6 ft W × 8 ft H = 480 ft³
- ACH: 6
- CFM = (480 × 6) / 60 = 48 CFM minimum

**Note:** Size fan for 1.5× calculated minimum to allow headroom and seasonal adjustment.

### Pressure Control
- **Target:** Slight positive pressure (+0.02 to +0.05" WC inside oven)
- **Instrumentation:** Differential pressure sensor (Magnehelic or digital transmitter)
- **Control:** VFD on exhaust motor; manual damper as backup

### Makeup Air
- **Purpose:** Replace exhausted air to avoid negative pressure
- **Source:** Controlled intake louver or dedicated makeup air unit
- **Heating:** Optional preheat (heat exchanger or tempering unit) to reduce thermal shock

---

## Airflow Pattern Design

### Bottom-Up Configuration (Recommended)
- **Supply Plenums:** Below conveyor path, full length of oven
- **Diffusers:** Perforated plates or louvered grilles to distribute air evenly
- **Velocity:** 200–400 FPM at product level (high enough for heat transfer, low enough to avoid turbulence)
- **Recirculation:** 80–90% of air recirculated; 10–20% exhausted and replaced with makeup

### Burner/Heater Integration
- **Gas Burners:** Indirect-fired (heat exchanger) to avoid flame contact with product or powder residue
- **Electric Heaters:** Finned elements in recirculation ductwork (safer for powder environment)
- **Temperature Control:** PID controller with RTD sensors at multiple zones

### Fan Sizing (Recirculation)
```
CFM_recirc = Oven Volume [ft³] × 30–60 air changes/hour (for uniform heating)
```
Example:
- Oven: 480 ft³
- ACH: 40 (mid-range)
- CFM = (480 × 40) / 60 = 320 CFM recirculation fan

---

## Baffles and Physical Seals

### Internal Fixed Baffles
- **Material:** 16-gauge stainless steel or aluminized steel
- **Placement:** 12"–18" inside oven from entrance/exit openings
- **Angle:** 30–45° downward slope to redirect rising hot air toward conveyor path
- **Clearance:** Minimum 6" above tallest product + hanger

### High-Temperature Strip Curtains
- **Material:** 
  - Fiberglass cloth (1000°F rated) for 400°F+ ovens
  - Silicone-coated fabric (500°F rated) for lower-temp applications
- **Strip Width:** 8"–12" overlap; 50% overlap pattern
- **Mounting:** Stainless steel track with quick-release clips for easy replacement
- **Placement:** At vestibule-to-oven interface (secondary seal)

---

## Control and Instrumentation

### Temperature Control
- **Zones:** Minimum 2 zones (preheat, cure); 3–4 zones for longer ovens
- **Sensors:** Type K or J thermocouples; RTDs for tighter control
- **Controller:** PID with auto-tune; data logging capability
- **Setpoint:** Typical 375–400°F for powder cure (verify powder TDS)

### Air Seal Control
- **VFD:** 0–60 Hz range on air curtain blower motors
- **Feedback:** Optional air velocity sensor (pitot tube or thermal anemometer) for closed-loop control
- **Setpoint:** User-adjustable via HMI (1000–2500 FPM range)

### Exhaust Control
- **VFD:** 0–60 Hz on exhaust fan
- **Pressure Feedback:** Magnehelic or transmitter with 4–20mA output
- **Interlock:** Exhaust must run whenever oven is energized (NFPA 86)

### Safety Interlocks
- **High-Limit Thermostat:** 450°F cutoff (adjustable based on powder spec)
- **Airflow Proving Switch:** Confirms recirculation fan operation before burner ignition
- **Exhaust Proving Switch:** Confirms exhaust airflow before burner ignition
- **Emergency Stop:** Hardwired e-stop kills all power (burner, fans, conveyor)

---

## Energy Efficiency Targets

### Baseline (No Air Seals or Vestibules)
- Heat loss at openings: ~30–50% of total oven energy consumption
- Overall efficiency: 50–60%

### Optimized Design (Air Seals + Vestibule + Controlled Exhaust)
- Heat loss at openings: ~10–20%
- Overall efficiency: 75–85%
- **Energy Savings:** 40–60% reduction in fuel/electricity vs. baseline

### Payback Period
- Typical: 1–3 years (depends on throughput, energy costs, and capital investment)
- High-volume operations: <1 year payback common

---

## Product Recommendations (Commercial Solutions)

### Air Curtain Units
- **Berner International:** AirStream Series (industrial air doors, customizable for high-temp)
- **Mars Air Systems:** Heavy-duty air curtains with VFD options
- **Exair:** Compressed air-driven Super Air Knife (low CFM, very high velocity; best for small openings)

### High-Temperature Strip Curtains
- **TMI (Total Material Innovations):** Fiberglass and silicone curtains rated to 1000°F
- **Aleco:** High-temp vinyl and silicone strips; modular mounting systems
- **Custom Fabrication:** Local metal fab shop can make stainless mesh curtains for extreme temps

### Insulation Panels
- **Metal-Fab:** Pre-fab insulated panels for oven walls (mineral wool core, steel skin)
- **Rockwool (Roxul):** Industrial board insulation (DIY assembly)
- **Insultherm:** Ceramic fiber modules and blankets (for very high-temp applications >1000°F)

### VFD (Variable Frequency Drives)
- **ABB:** ACS355 series (1–3 HP range, NEMA 1 or 4X)
- **Siemens:** Sinamics G120C (cost-effective, simple programming)
- **Allen-Bradley (Rockwell):** PowerFlex 525 (robust, good integration with PLCs)

---

## Next Steps: Detailed Design

To proceed to fabrication-ready drawings, provide:

1. **Throughput Requirements:** Parts per hour, hanger spacing, conveyor speed
2. **Product Envelope:** Max width, height, weight (determines conveyor rating and oven clearance)
3. **Cure Schedule:** Dwell time at temperature (sets oven length)
4. **Powder Specification:** Cure temp, cure time, VOC data (affects exhaust sizing)
5. **Available Floor Space:** L × W footprint limit (affects vestibule length and oven layout)
6. **Utilities:** Gas pressure, electrical service (208/240/480V, 3-phase?), compressed air availability
7. **Budget Range:** For vendor selection and design complexity decisions

**Deliverables from Next Phase:**
- Mechanical layout drawing (plan and elevation views)
- Air seal nozzle detail and blower specification
- Insulation assembly drawing with R-value calculations
- Electrical single-line diagram and control panel layout
- Bill of Materials (BOM) with vendor part numbers
- Energy consumption estimate and ROI analysis

---

**Questions for Further Elaboration:**
1. Do you want detailed air seal velocity calculations (including powder blow-off risk analysis)?
2. Should I create a thermal loss model (spreadsheet or Python script) to estimate energy savings for your specific oven size?
3. Do you need CAD sketches or 3D model recommendations for the vestibule and baffle layout?
