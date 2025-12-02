#!/usr/bin/env python3
"""
Generic Support & Resistance Analyzer (Daily)

Usage:
  python support_resistance.py AAPL

- Fetches daily OHLC from Stooq
- Computes classic pivots, recent swing highs/lows, 20/50-day ranges,
  and moving average pivots (SMA20/50/200)
- Writes <symbol>_support_resistance.md in this folder
"""
from __future__ import annotations
import sys
import csv
from datetime import datetime, timezone
from pathlib import Path
from urllib.request import urlopen, Request
from urllib.error import URLError, HTTPError

ROOT = Path(__file__).resolve().parent


def fetch_ohlc_stooq(symbol: str, days: int = 400):
    sym = f"{symbol.lower()}.us"
    url = f"https://stooq.com/q/d/l/?s={sym}&i=d"
    try:
        with urlopen(Request(url, headers={"User-Agent": "APM-SR/1.0"}), timeout=20) as r:
            text = r.read().decode("utf-8", errors="replace")
        rows = list(csv.DictReader(text.splitlines()))
        data = []
        for r in rows:
            if r.get("Close") and r["Close"] != "N/D":
                try:
                    data.append({
                        "date": r["Date"],
                        "open": float(r["Open"]),
                        "high": float(r["High"]),
                        "low": float(r["Low"]),
                        "close": float(r["Close"]),
                        "volume": float(r.get("Volume", 0) or 0),
                    })
                except ValueError:
                    continue
        return data[-days:] if len(data) >= 30 else None
    except (URLError, HTTPError):
        return None


def sma(seq, period):
    if len(seq) < period:
        return None
    return sum(seq[-period:]) / period


def recent_range(values, n):
    if len(values) < n:
        return None, None
    sub = values[-n:]
    return min(sub), max(sub)


def classic_pivots(prev_high, prev_low, prev_close):
    p = (prev_high + prev_low + prev_close) / 3
    r1 = 2 * p - prev_low
    s1 = 2 * p - prev_high
    r2 = p + (prev_high - prev_low)
    s2 = p - (prev_high - prev_low)
    r3 = prev_high + 2 * (p - prev_low)
    s3 = prev_low - 2 * (prev_high - p)
    return {"P": p, "R1": r1, "S1": s1, "R2": r2, "S2": s2, "R3": r3, "S3": s3}


def swing_points(closes, lookback=3):
    last_low = None
    last_high = None
    for i in range(lookback, len(closes) - lookback):
        window = closes[i - lookback:i + lookback + 1]
        c = closes[i]
        if c == min(window):
            last_low = c
        if c == max(window):
            last_high = c
    return last_low, last_high


def fmt(x):
    return f"{x:.2f}" if x is not None else "n/a"


def write_md(sym: str, ohlc: list[dict]):
    closes = [d["close"] for d in ohlc]
    price = closes[-1]
    s20 = sma(closes, 20)
    s50 = sma(closes, 50)
    s200 = sma(closes, 200)

    low20, high20 = recent_range(closes, 20)
    low50, high50 = recent_range(closes, 50)

    prev = ohlc[-2]
    piv = classic_pivots(prev["high"], prev["low"], prev["close"])
    swing_low, swing_high = swing_points(closes, lookback=3)

    out_file = ROOT / f"{sym.lower()}_support_resistance.md"
    lines = [
        f"# {sym} Support & Resistance",
        "",
        f"Generated: {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')}",
        "",
        f"- Price: {fmt(price)}",
        f"- SMA20 / SMA50 / SMA200: {fmt(s20)} / {fmt(s50)} / {fmt(s200)}",
        f"- 20D Range: {fmt(low20)} - {fmt(high20)}",
        f"- 50D Range: {fmt(low50)} - {fmt(high50)}",
        "",
        "## Classic Pivots (from prior day)",
        f"- P: {fmt(piv['P'])}",
        f"- R1 / S1: {fmt(piv['R1'])} / {fmt(piv['S1'])}",
        f"- R2 / S2: {fmt(piv['R2'])} / {fmt(piv['S2'])}",
        f"- R3 / S3: {fmt(piv['R3'])} / {fmt(piv['S3'])}",
        "",
        "## Recent Swings (fractal)",
        f"- Last swing high: {fmt(swing_high)}",
        f"- Last swing low: {fmt(swing_low)}",
        "",
        "## Practical Read",
        "- Resistance: prior swing high, 20D/50D highs, pivot R1/R2.",
        "- Support: SMA20/SMA50, prior swing low, 20D/50D lows, pivot S1/S2.",
    ]
    out_file.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {out_file}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python support_resistance.py <SYMBOL>")
        return 2
    sym = sys.argv[1].upper()
    ohlc = fetch_ohlc_stooq(sym)
    if not ohlc or len(ohlc) < 60:
        print("Failed to fetch enough data from Stooq.")
        return 1
    write_md(sym, ohlc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
