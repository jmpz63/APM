#!/usr/bin/env python3
"""Generate portfolio_snapshot.md from positions.json with live quotes.

Stocks via Stooq CSV (symbol.us), BTC via CoinGecko. No API keys required.
"""
from __future__ import annotations
import csv
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from urllib.request import urlopen, Request
from urllib.error import URLError, HTTPError

ROOT = Path(__file__).resolve().parent
POS_FILE = ROOT / "positions.json"
OUT_FILE = ROOT / "portfolio_snapshot.md"


def fetch_stooq_quote(symbol: str) -> float | None:
    # Stooq US tickers often require .us suffix
    candidates = [f"{symbol.lower()}.us", symbol.lower()]
    for sym in candidates:
        url = f"https://stooq.com/q/l/?s={sym}&f=sd2t2ohlcv&h&e=csv"
        try:
            with urlopen(Request(url, headers={"User-Agent": "APM-Portfolio/1.0"}), timeout=15) as r:
                text = r.read().decode("utf-8", errors="replace")
            rows = list(csv.DictReader(text.splitlines()))
            if rows and rows[0].get("Close") not in (None, "N/D"):
                return float(rows[0]["Close"])
        except (URLError, HTTPError, ValueError):
            continue
    return None


def fetch_btc_usd() -> float | None:
    url = "https://api.coingecko.com/api/v3/simple/price?ids=bitcoin&vs_currencies=usd"
    try:
        with urlopen(Request(url, headers={"User-Agent": "APM-Portfolio/1.0"}), timeout=15) as r:
            data = json.loads(r.read().decode("utf-8"))
        return float(data.get("bitcoin", {}).get("usd"))
    except Exception:
        return None


def fmt_usd(x: float | None) -> str:
    return f"${x:,.2f}" if x is not None else "n/a"


def main():
    if not POS_FILE.exists():
        print("positions.json not found")
        return 1
    positions = json.loads(POS_FILE.read_text(encoding="utf-8"))

    ts = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    lines = [f"# Portfolio Snapshot", "", f"Generated: {ts}", ""]

    total_value = 0.0
    total_cost = 0.0

    # Equities
    if positions.get("equities"):
        lines.append("## Equities")
        for eq in positions["equities"]:
            sym = eq["symbol"].upper()
            lots = eq.get("lots", [])
            shares = sum(lot["shares"] for lot in lots)
            cost = sum(lot["shares"] * float(lot["price"]) for lot in lots)
            avg = (cost / shares) if shares else 0.0
            price = fetch_stooq_quote(sym)
            mv = (price or 0.0) * shares
            pl = mv - cost if price is not None else None
            if price is not None:
                total_value += mv
            total_cost += cost
            lines.append(f"- {sym}: {shares} sh @ {fmt_usd(avg)} | Price: {fmt_usd(price)} | MV: {fmt_usd(mv if price else None)} | UPL: {fmt_usd(pl)}")
        lines.append("")

    # Crypto
    if positions.get("crypto"):
        lines.append("## Crypto")
        for c in positions["crypto"]:
            sym = c["symbol"].upper()
            lots = c.get("lots", [])
            units = sum(lot["units"] for lot in lots)
            cost = sum(lot["units"] * float(lot["price"]) for lot in lots)
            avg = (cost / units) if units else 0.0
            price = fetch_btc_usd() if sym == "BTC-USD" else None
            mv = (price or 0.0) * units
            pl = mv - cost if price is not None else None
            if price is not None:
                total_value += mv
            total_cost += cost
            lines.append(f"- {sym}: {units:.8f} @ {fmt_usd(avg)} | Price: {fmt_usd(price)} | MV: {fmt_usd(mv if price else None)} | UPL: {fmt_usd(pl)}")
        lines.append("")

    # Options (summary only; detailed payoff via options_helper.py)
    if positions.get("options"):
        lines.append("## Options (summary)")
        for opt in positions["options"]:
            und = opt["underlying"].upper()
            side = opt["side"]
            strike = opt["strike"]
            expiry = opt["expiry"]
            c = opt.get("contracts", 1)
            prem = opt.get("premium", 0.0)
            if side == "long_put":
                lines.append(f"- LONG PUT {und} {strike} {expiry} x{c} | Paid {fmt_usd(prem)} per share | Breakeven: {fmt_usd(strike - prem)}")
            elif side == "short_put":
                lines.append(f"- SHORT PUT {und} {strike} {expiry} x{c} | Received {fmt_usd(prem)} per share | Breakeven: {fmt_usd(strike - prem)}")
        lines.append("")

    lines.append("## Totals")
    lines.append(f"- Cost Basis (equities+crypto): {fmt_usd(total_cost)}")
    lines.append(f"- Market Value (equities+crypto): {fmt_usd(total_value)}")
    if total_value is not None:
        lines.append(f"- Unrealized P/L: {fmt_usd(total_value - total_cost)}")

    OUT_FILE.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {OUT_FILE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())