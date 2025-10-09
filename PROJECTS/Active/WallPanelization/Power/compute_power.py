#!/usr/bin/env python3
"""
Compute aggregate power from power_budget.csv and provide a rough generator sizing hint.
No external deps. CSV columns: Component,Subsystem,Qty,Voltage,Phase,FullLoadCurrentA,NameplateKW,TypicalKW,SurgeKW,DutyCycle,Notes
"""
import csv
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CSV_PATH = ROOT / "power_budget.csv"
PF = 0.85  # assumed power factor when only kW are provided
DERATE = 1.25  # altitude/temp headroom


def parse_float(s):
    try:
        return float(s)
    except Exception:
        return 0.0


def main():
    if not CSV_PATH.exists():
        print("Missing power_budget.csv")
        return 1

    total_typical_kw = 0.0
    total_peak_kw = 0.0

    with CSV_PATH.open(newline='', encoding='utf-8') as f:
        r = csv.DictReader(f)
        for row in r:
            qty = int(parse_float(row.get('Qty', '1')))
            typical_kw = parse_float(row.get('TypicalKW', '0')) * qty
            surge_kw = parse_float(row.get('SurgeKW', '0')) * qty
            duty = parse_float(row.get('DutyCycle', '1'))
            total_typical_kw += typical_kw * duty
            total_peak_kw += surge_kw  # naive coincidence; adjust if needed

    kva_cont = total_typical_kw / PF
    kva_peak = total_peak_kw / PF if total_peak_kw else kva_cont
    kva_req = max(kva_cont, kva_peak) * DERATE

    print(f"Continuous kW (duty-weighted): {total_typical_kw:.2f} kW")
    print(f"Peak kW (coincident est.):    {total_peak_kw:.2f} kW")
    print(f"Req kVA with derate:          {kva_req:.2f} kVA (PF {PF}, derate {DERATE}x)")
    print("Select generator with standby kVA >= Req and continuous >= kVA_cont.")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
