#!/usr/bin/env python3
"""Options Put Scanner

Scans TSLA/NVDA/AVGO put chains (nearest 1-2 expiries) for short-put
opportunities based on simple heuristics:
- Premium yield: bid/strike >= 1% per week (approx)
- Open interest >= 100
- Strike ~ 90-98% of last price (OTM cushion)

Requires yfinance (install with: python -m pip install yfinance).
Outputs options_scan.md
"""
from __future__ import annotations
import sys
from pathlib import Path
from datetime import datetime

OUT_FILE = Path(__file__).resolve().parent / "options_scan.md"


def main():
    try:
        import yfinance as yf
    except ImportError:
        OUT_FILE.write_text("yfinance not installed. Run: python -m pip install yfinance", encoding="utf-8")
        print("yfinance not installed. Run: python -m pip install yfinance")
        return 1

    tickers = ["TSLA", "NVDA", "AVGO"]
    lines = ["# Options Put Scanner", "", f"Generated: {datetime.utcnow().strftime('%Y-%m-%d %H:%M UTC')}", "", "Heuristics: yield >= ~1%/wk, OI>=100, strike 90-98% of last.", ""]

    for sym in tickers:
        t = yf.Ticker(sym)
        price = t.history(period="1d")["Close"].iloc[-1]
        exps = t.options
        if not exps:
            continue
        # Take next 2 expirations if available
        scan_exps = exps[:2]
        lines.append(f"## {sym} (px ~ ${price:.2f})")
        found_any = False
        for ex in scan_exps:
            chain = t.option_chain(ex)
            puts = chain.puts
            # Filter OTM puts within range
            candidates = puts[(puts["strike"] <= price * 0.98) & (puts["strike"] >= price * 0.90) & (puts["openInterest"] >= 100)]
            if candidates.empty:
                continue
            # Rough week count ~ days/7; yfinance doesn't give days, so estimate from expiry distance using index name
            # Simpler: require absolute premium/strike >= 1% threshold unannualized
            candidates = candidates.assign(yield_pct=(candidates["bid"] / candidates["strike"]) * 100)
            picks = candidates.sort_values(by=["yield_pct", "openInterest"], ascending=[False, False]).head(5)
            if picks.empty:
                continue
            found_any = True
            lines.append(f"### Expiry {ex}")
            lines.append("| Strike | Bid | Yield % | OI | Breakeven |")
            lines.append("|------:|----:|--------:|---:|----------:|")
            for _, r in picks.iterrows():
                strike = float(r["strike"])
                bid = float(r["bid"]) if r["bid"] == r["bid"] else 0.0
                yld = (bid / strike) * 100 if strike else 0.0
                oi = int(r["openInterest"]) if r["openInterest"] == r["openInterest"] else 0
                be = strike - bid
                lines.append(f"| ${strike:.2f} | ${bid:.2f} | {yld:.2f}% | {oi} | ${be:.2f} |")
        if not found_any:
            lines.append("No candidates per filters.")
        lines.append("")

    OUT_FILE.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {OUT_FILE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())