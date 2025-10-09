# Generator Sizing (Draft)

Inputs (update from power_budget.csv and duty cycles):
- Continuous kW (Σ TypicalKW × DutyCycle)
- Peak kW (Σ SurgeKW over coincident loads)
- Power factor (assume 0.85 unless measured)

Calcs:
- kVA_cont = kW_cont / PF
- kVA_peak = kW_peak / PF
- Apply derating factors (altitude, temp): kVA_req = max(kVA_cont, kVA_peak) × derate
- Select generator with standby kVA ≥ kVA_req and continuous kVA ≥ kVA_cont

Notes:
- Consider motor starting: VFDs reduce inrush; across-the-line increases surge.
- Include 20–30% headroom for transients and future growth.
- Fuel sizing for expected runtime hours; include refueling logistics.
