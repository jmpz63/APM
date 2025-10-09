# Power Planning (Field-Deployable)

Goal: Capture total electrical load, generator sizing, distribution, and safety for the WallPanelization system.

Deliverables:
- power_budget.csv — rolling power inventory (nameplate, typical, surge, duty cycle)
- generator_sizing.md — sizing calcs (kW/kVA), deratings (altitude/temp), fuel
- single_line.drawing.md — narrative single-line until CAD is attached
- layout_power.drawing.md — site layout concepts for genset, distribution, and cable routing
- standards.md — NEC/IEC references, grounding/bonding approach, GFCI/AFCI strategy

Assumptions to refine:
- Site power not guaranteed → primary generator power
- Environmental: outdoor operation, temp range TBD
- Power quality: target ±5% voltage, THD < 5%
