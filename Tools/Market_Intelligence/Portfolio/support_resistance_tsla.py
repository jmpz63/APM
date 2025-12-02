#!/usr/bin/env python3
"""
TSLA Support & Resistance Analyzer (Daily)

- Fetches daily OHLCV from Stooq
- Computes classic pivot points, recent swing highs/lows, 20/50-day ranges,
  and moving average pivots (SMA20/50/200)
- Outputs a concise markdown file: tsla_support_resistance.md

Stdlib only. Uses Stooq free CSV; data is end-of-day.
"""
from __future__ import annotations
import csv
from datetime import datetime, timezone
from pathlib import Path
from urllib.request import urlopen, Request
from urllib.error import URLError, HTTPError
from statistics import mean

ROOT = Path(__file__).resolve().parent
OUT_FILE = ROOT / "tsla_support_resistance.md"


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


def swing_points(closes, lookback=5):
    """Return last swing low and swing high by simple fractal method."""
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


def write_md(sym: str, ohlc: list[dict]):
    closes = [d["close"] for d in ohlc]
    highs = [d["high"] for d in ohlc]
    lows = [d["low"] for d in ohlc]

    price = closes[-1]
    s20 = sma(closes, 20)
    s50 = sma(closes, 50)
    s200 = sma(closes, 200)

    low20, high20 = recent_range(closes, 20)
    low50, high50 = recent_range(closes, 50)

    # Classic pivots from prior day
    prev = ohlc[-2]
    piv = classic_pivots(prev["high"], prev["low"], prev["close"])

    swing_low, swing_high = swing_points(closes, lookback=3)

    def fmt(x):
        return f"{x:.2f}" if x is not None else "n/a"

    lines = [
        f"# TSLA Support & Resistance",
        "",
        f"Generated: {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')}",
        "",
        f"- Price: {fmt(price)}",
        f"- SMA20 / SMA50 / SMA200: {fmt(s20)} / {fmt(s50)} / {fmt(s200)}",
        f"- 20D Range: {fmt(low20)} - {fmt(high20)}",
        f"- 50D Range: {fmt(low50)} - {fmt(high50)}",
        "",
        "## Classic Pivots (from prior day)",
        f"- P: {piv['P']:.2f}",
        f"- R1 / S1: {piv['R1']:.2f} / {piv['S1']:.2f}",
        f"- R2 / S2: {piv['R2']:.2f} / {piv['S2']:.2f}",
        f"- R3 / S3: {piv['R3']:.2f} / {piv['S3']:.2f}",
        "",
        "## Recent Swings (fractal)",
        f"- Last swing high: {fmt(swing_high)}",
        f"- Last swing low: {fmt(swing_low)}",
        "",
        "## Practical Read",
        "- Resistance: prior swing high, 20D/50D highs, pivot R1/R2.",
        "- Support: SMA20/SMA50, prior swing low, 20D/50D lows, pivot S1/S2.",
    ]

    OUT_FILE.write_text("\n".join(lines), encoding="utf-8")


def main():
    ohlc = fetch_ohlc_stooq("TSLA")
    if not ohlc or len(ohlc) < 60:
        print("Failed to fetch enough data from Stooq.")
        return 1
    write_md("TSLA", ohlc)
    print(f"Wrote {OUT_FILE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
