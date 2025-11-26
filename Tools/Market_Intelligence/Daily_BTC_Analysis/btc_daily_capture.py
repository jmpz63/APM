"""Daily BTC Data Capture & Markdown Generator

Fetches recent BTC market data from CoinGecko and computes a set of technical
indicators. Produces two files in the same directory:

1. YYYY-MM-DD_metrics.json  - Raw metrics & indicators
2. YYYY-MM-DD_enhanced.md   - Markdown summary using enhanced template

Run once per day (e.g., via scheduled task). Requires only stdlib.
"""
from __future__ import annotations
import json
import math
import statistics
from datetime import datetime, timezone
from pathlib import Path
from urllib.request import urlopen, Request
from urllib.error import URLError, HTTPError

OUT_DIR = Path(__file__).resolve().parent
COINGECKO_CHART_URL = "https://api.coingecko.com/api/v3/coins/bitcoin/market_chart?vs_currency=usd&days=200"
COINGECKO_PRICE_URL = "https://api.coingecko.com/api/v3/simple/price?ids=bitcoin&vs_currencies=usd"

def fetch_json(url: str):
    try:
        req = Request(url, headers={"User-Agent": "APM-BTC-Capture/1.0"})
        with urlopen(req, timeout=20) as r:
            return json.loads(r.read().decode("utf-8"))
    except (URLError, HTTPError):
        return None

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
        change = values[i] - values[i - 1]
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
    if len(values) < 50:
        return None
    ema12 = ema(values, 12)
    ema26 = ema(values, 26)
    macd_line = [a - b for a, b in zip(ema12[-len(ema26):], ema26)]
    signal = ema(macd_line, 9)
    histogram = macd_line[-1] - signal[-1]
    return {"macd": macd_line[-1], "signal": signal[-1], "histogram": histogram}

def realized_vol(prices, window=7):
    if len(prices) < window + 1:
        return None
    returns = []
    for i in range(1, window + 1):
        returns.append(math.log(prices[-i] / prices[-i - 1]))
    stdev = statistics.pstdev(returns)
    # Annualize using sqrt(365)
    return stdev * math.sqrt(365)

def sma(values, period):
    if len(values) < period:
        return None
    return sum(values[-period:]) / period

def generate():
    chart = fetch_json(COINGECKO_CHART_URL)
    spot = fetch_json(COINGECKO_PRICE_URL)
    now = datetime.now(timezone.utc)

    if not chart or "prices" not in chart:
        return {"error": "chart_unavailable", "timestamp": now.isoformat()}

    # CoinGecko returns list of [ms_timestamp, price]
    prices = [p[1] for p in chart["prices"]]
    closing_prices = prices  # Approximation (chart endpoint granularity varies)

    metrics = {
        "timestamp": now.isoformat(),
        "current_price": spot.get("bitcoin", {}).get("usd"),
        "sma20": sma(closing_prices, 20),
        "sma50": sma(closing_prices, 50),
        "sma200": sma(closing_prices, 200),
        "rsi14": rsi(closing_prices[-15:]),
        "macd": macd(closing_prices),
        "realized_vol_7d": realized_vol(closing_prices, 7),
        "price_distance_sma200_pct": None,
    }
    if metrics["current_price"] and metrics["sma200"]:
        metrics["price_distance_sma200_pct"] = ((metrics["current_price"] - metrics["sma200"]) / metrics["sma200"]) * 100

    return metrics

def write_outputs(data):
    date_str = datetime.utcnow().strftime("%Y-%m-%d")
    json_path = OUT_DIR / f"{date_str}_metrics.json"
    md_path = OUT_DIR / f"{date_str}_enhanced.md"

    json_path.write_text(json.dumps(data, indent=2), encoding="utf-8")

    lines = [f"# Daily BTC Enhanced Analysis: {date_str}", "", f"**Timestamp:** {data.get('timestamp')}"]
    if "error" in data:
        lines.append(f"Data unavailable: {data['error']}")
    else:
        lines.extend([
            f"**Current Price:** ${data.get('current_price')}",
            f"**SMA20 / SMA50 / SMA200:** {data.get('sma20')} / {data.get('sma50')} / {data.get('sma200')}",
            f"**RSI(14):** {round(data.get('rsi14'),2) if data.get('rsi14') else 'n/a'}",
            f"**MACD (line/signal/hist):** {data['macd']['macd']:.2f} / {data['macd']['signal']:.2f} / {data['macd']['histogram']:.2f}" if data.get('macd') else "**MACD:** n/a",
            f"**Realized Vol (7d, annualized):** {round(data.get('realized_vol_7d'),4) if data.get('realized_vol_7d') else 'n/a'}",
            f"**Distance from SMA200 (%):** {round(data.get('price_distance_sma200_pct'),2) if data.get('price_distance_sma200_pct') else 'n/a'}",
            "",
            "## Scenario Probabilities (Initial Placeholder)",
            "- Bullish Continuation: 33%", 
            "- Major Correction: 33%", 
            "- Range Compression: 34%", 
            "\n(Adjust after multiple days of data aggregation.)",
            "",
            "## Risk Flags (Heuristics Placeholder)",
            "- Insufficient multi-day confluence established yet.",
            "",
            "## Invalidation Levels (Placeholder)",
            "- Bullish bias invalid below SMA50 if RSI < 45.",
            "- Bearish acceleration invalid if price holds > SMA20 for 3 consecutive closes.",
        ])
    md_path.write_text("\n".join(lines), encoding="utf-8")
    return json_path, md_path

def main():
    data = generate()
    j, m = write_outputs(data)
    print(f"Wrote: {j.name}, {m.name}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())