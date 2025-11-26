#!/usr/bin/env python3
"""Options payoff helper for portfolio positions.

Generates payoff tables at expiry for listed options in positions.json.
Focus on: AVGO 11/28 392.5P (long), TSLA 11/28 417.5P (short).
"""
from __future__ import annotations
import json
from pathlib import Path
from datetime import datetime

ROOT = Path(__file__).resolve().parent
POS_FILE = ROOT / "positions.json"
OUT_FILE = ROOT / "options_payoff.md"


def payoff_long_put(underlying_price: float, strike: float, premium: float, contracts: int) -> float:
    intrinsic = max(strike - underlying_price, 0.0)
    return (intrinsic - premium) * 100 * contracts


def payoff_short_put(underlying_price: float, strike: float, premium: float, contracts: int) -> float:
    intrinsic = max(strike - underlying_price, 0.0)
    return (premium - intrinsic) * 100 * contracts


def table_for_option(opt: dict) -> list[str]:
    und = opt["underlying"].upper()
    side = opt["side"]
    strike = float(opt["strike"])
    prem = float(opt["premium"])
    c = int(opt.get("contracts", 1))
    expiry = opt["expiry"]

    # Build a price grid around strike
    # Wider band for AVGO due to price level
    steps = []
    # 60% to 140% of strike in 10% increments
    for pct in [0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4]:
        steps.append(round(strike * pct, 2))

    lines = [f"### {und} {expiry} {strike} {('PUT' if 'put' in side else 'CALL')} ({'LONG' if 'long' in side else 'SHORT'})",
             "| Price | P/L at Expiry |",
             "|------:|-------------:|"]
    for price in steps:
        if side == "long_put":
            pl = payoff_long_put(price, strike, prem, c)
        elif side == "short_put":
            pl = payoff_short_put(price, strike, prem, c)
        else:
            continue
        lines.append(f"| ${price:,.2f} | ${pl:,.0f} |")

    lines.append("")
    lines.append(f"- Breakeven: ${strike - prem:,.2f}")
    if side == "long_put":
        lines.append(f"- Max loss: ${prem * 100 * c:,.0f} | Max gain: approx ${strike * 100 * c:,.0f} if {und}->0")
    else:
        lines.append(f"- Max gain: ${prem * 100 * c:,.0f} | Max loss: large if {und}->0 (cash-secured advised)")
    lines.append("")
    return lines


def main():
    pos = json.loads(POS_FILE.read_text(encoding="utf-8"))
    opts = pos.get("options", [])
    lines = [f"# Options Payoff Scenarios", "", f"Generated: {datetime.utcnow().strftime('%Y-%m-%d %H:%M UTC')}", ""]
    for opt in opts:
        if opt["side"] in ("long_put", "short_put"):
            lines.extend(table_for_option(opt))

    OUT_FILE.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {OUT_FILE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())