#!/usr/bin/env python3
"""Market Trend Scanner

Fetches daily history via Stooq for a set of tickers and identifies
"bull-run" conditions:
- price > SMA50
- SMA50 > SMA200
- RSI(14) > 55
- MACD histogram > 0

Outputs market_scan.md with signals and key levels.
"""
from __future__ import annotations
import csv
from dataclasses import dataclass
from pathlib import Path
from urllib.request import urlopen, Request
from urllib.error import URLError, HTTPError
from datetime import datetime, timezone
import math
import statistics
import json

ROOT = Path(__file__).resolve().parent
OUT_FILE = ROOT / "market_scan.md"
POS_FILE = ROOT / "positions.json"


def fetch_history_stooq(symbol: str, days: int = 250):
    sym = f"{symbol.lower()}.us"
    url = f"https://stooq.com/q/d/l/?s={sym}&i=d"
    try:
        with urlopen(Request(url, headers={"User-Agent": "APM-Scanner/1.0"}), timeout=20) as r:
            text = r.read().decode("utf-8", errors="replace")
        rows = list(csv.DictReader(text.splitlines()))
        closes = [float(r["Close"]) for r in rows if r.get("Close") and r["Close"] != "N/D"]
        return closes[-days:] if len(closes) >= 10 else None
    except (URLError, HTTPError, ValueError):
        return None


def sma(values, period):
    if len(values) < period:
        return None
    return sum(values[-period:]) / period


def ema(values, period):
    if not values:
        return []
    k = 2 / (period + 1)
    ema_vals = []
    prev = values[0]
    for v in values:
        prev = (v - prev) * k + prev
        ema_vals.append(prev)
    return ema_vals


def rsi(values, period=14):
    if len(values) < period + 1:
        return None
    gains = []
    losses = []
    for i in range(1, period + 1):
        change = values[-(i)] - values[-(i+1)]
        if change >= 0:
            gains.append(change)
        else:
            losses.append(-change)
    avg_gain = sum(gains) / period if gains else 0
    avg_loss = sum(losses) / period if losses else 0
    if avg_loss == 0:
        return 100.0
    rs = avg_gain / avg_loss
    return 100 - (100 / (1 + rs))


def macd(values):
    if len(values) < 35:
        return None
    ema12 = ema(values, 12)
    ema26 = ema(values, 26)
    macd_line = [a - b for a, b in zip(ema12[-len(ema26):], ema26)]
    signal = ema(macd_line, 9)
    return macd_line[-1] - signal[-1]


def swing_levels(values, window=20):
    recent = values[-window:]
    return min(recent), max(recent)


def main():
    # Tickers from positions + watchlist
    tickers = set()
    if POS_FILE.exists():
        pos = json.loads(POS_FILE.read_text(encoding="utf-8"))
        for eq in pos.get("equities", []):
            tickers.add(eq["symbol"].upper())
    tickers.update({"TSLA", "NVDA", "AVGO"})

    lines = ["# Market Trend Scan", "", f"Generated: {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')}", ""]
    bullish = []
    neutrals = []
    failures = []

    for sym in sorted(tickers):
        data = fetch_history_stooq(sym)
        if not data:
            failures.append(sym)
            continue
        price = data[-1]
        s50 = sma(data, 50)
        s200 = sma(data, 200)
        r = rsi(data, 14)
        h = macd(data)
        low, high = swing_levels(data, 20)

        cond = (s50 and s200 and r is not None and h is not None and
                price > s50 and s50 > s200 and r > 55 and h > 0)
        entry = f"- {sym}: Px {price:.2f} | SMA50 {s50:.2f} | SMA200 {s200:.2f} | RSI {r:.1f} | MACDhist {h:.2f} | 20d L/H {low:.2f}/{high:.2f}"
        if cond:
            bullish.append(entry)
        else:
            neutrals.append(entry)

    if bullish:
        lines.append("## Bull-Run Candidates")
        lines.extend(bullish)
        lines.append("")

    if neutrals:
        lines.append("## Others (Not All Signals Aligned)")
        lines.extend(neutrals)
        lines.append("")

    if failures:
        lines.append("## Fetch Failures")
        lines.append(", ".join(sorted(failures)))

    OUT_FILE.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {OUT_FILE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())