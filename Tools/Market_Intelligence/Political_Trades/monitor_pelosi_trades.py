"""
Pelosi Trade Monitor (Polling)

Purpose:
- Poll a configured provider endpoint (JSON) for recent congressional trades
- Filter for Nancy/Paul Pelosi
- Append new items to politician_trades.md and update a local state file to avoid duplicates

Notes:
- There is NO true real-time for Congressional trades; reports are filed days/weeks after trades.
- This script alerts within minutes of a filing being posted to your chosen data provider.

Configuration order of precedence:
1) Environment variables (PROVIDER_URL, PROVIDER_HEADERS_JSON, POLITICIAN_NAMES)
2) provider_config.json in this folder (see example file)

Run:
  python monitor_pelosi_trades.py

Schedule (Windows): use schedule_politician_monitor.ps1 in this folder.
"""
from __future__ import annotations
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List
from urllib.request import urlopen, Request

HERE = Path(__file__).resolve().parent
STATE_FILE = HERE / "monitor_state.json"
LOG_FILE = HERE / "politician_trades.md"
CONFIG_FILE = HERE / "provider_config.json"

DEFAULT_NAMES = ["Nancy Pelosi", "Paul Pelosi"]

# Helper: ISO parsing with graceful fallback

def parse_dt(s: str | None) -> datetime | None:
    if not s:
        return None
    for fmt in ("%Y-%m-%d", "%Y-%m-%dT%H:%M:%S%z", "%Y-%m-%dT%H:%M:%S.%f%z", "%Y-%m-%dT%H:%M:%SZ"):
        try:
            if fmt.endswith("Z"):
                # naive Zulu
                dt = datetime.strptime(s, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
            else:
                dt = datetime.strptime(s, fmt)
            return dt
        except Exception:
            continue
    return None


def load_config() -> Dict[str, Any]:
    cfg: Dict[str, Any] = {}
    # Try file
    if CONFIG_FILE.exists():
        try:
            cfg = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
        except Exception:
            cfg = {}
    # Env overrides
    if os.getenv("PROVIDER_URL"):
        cfg["provider_url"] = os.getenv("PROVIDER_URL")
    if os.getenv("PROVIDER_HEADERS_JSON"):
        try:
            cfg["headers"] = json.loads(os.getenv("PROVIDER_HEADERS_JSON", "{}"))
        except json.JSONDecodeError:
            pass
    if os.getenv("POLITICIAN_NAMES"):
        cfg["names"] = [n.strip() for n in os.getenv("POLITICIAN_NAMES", "").split(",") if n.strip()]
    return cfg


def http_get_json(url: str, headers: Dict[str, str] | None = None) -> Any:
    req = Request(url, headers=headers or {"User-Agent": "APM-PoliticalTrades/1.0"})
    with urlopen(req, timeout=25) as r:
        raw = r.read()
        return json.loads(raw.decode("utf-8", errors="ignore"))


def load_state() -> Dict[str, Any]:
    if STATE_FILE.exists():
        try:
            return json.loads(STATE_FILE.read_text(encoding="utf-8"))
        except Exception:
            return {}
    return {}


def save_state(state: Dict[str, Any]) -> None:
    STATE_FILE.write_text(json.dumps(state, indent=2), encoding="utf-8")


def append_log(lines: List[str]) -> None:
    if not LOG_FILE.exists():
        LOG_FILE.write_text("# Politician Trades Monitor\n\n", encoding="utf-8")
    with LOG_FILE.open("a", encoding="utf-8") as f:
        for line in lines:
            f.write(line.rstrip("\n") + "\n")


# Attempt to normalize common provider schemas
# Expected keys (best-effort):
#   - name (str) politician name
#   - ticker (str)
#   - transaction_date (str date)
#   - filed_date/report_date (str date)
#   - type (Buy/Sell)
#   - amount (str or number)
#   - link/url (str)

def normalize_item(item: Dict[str, Any]) -> Dict[str, Any]:
    name = item.get("name") or item.get("representative") or item.get("politician") or item.get("owner")
    ticker = (item.get("ticker") or item.get("asset") or item.get("symbol") or "").upper()
    tdate = item.get("transaction_date") or item.get("date") or item.get("trade_date")
    fdate = item.get("filed_date") or item.get("report_date") or item.get("disclosure_date")
    typ = item.get("type") or item.get("transaction") or item.get("direction")
    amt = item.get("amount") or item.get("amount_text") or item.get("range") or item.get("value")
    url = item.get("link") or item.get("url")
    return {
        "name": name,
        "ticker": ticker,
        "transaction_date": tdate,
        "filed_date": fdate,
        "type": typ,
        "amount": amt,
        "url": url,
        "_raw": item,
    }


def is_pelosi(name: str | None, targets: List[str]) -> bool:
    if not name:
        return False
    name_l = name.lower()
    return any(t.lower() in name_l for t in targets)


def main() -> int:
    cfg = load_config()
    provider_url = cfg.get("provider_url")
    headers = cfg.get("headers") or {}
    names = cfg.get("names") or DEFAULT_NAMES

    if not provider_url:
        # Provide guidance and exit cleanly
        appendix = (
            "\nNo provider_url configured. Create provider_config.json like:\n"
            "{\n  \"provider_url\": \"https://api.example.com/congress/trades/recent\",\n  \"headers\": {\"Authorization\": \"Bearer YOUR_KEY\"},\n  \"names\": [\"Nancy Pelosi\", \"Paul Pelosi\"]\n}\n"
        )
        print("Provider URL not set." + appendix)
        return 2

    try:
        data = http_get_json(provider_url, headers=headers)
    except Exception as e:
        print(f"Fetch failed: {e}")
        return 3

    # Expect list; if wrapped, attempt to unwrap
    items: List[Dict[str, Any]] = []
    if isinstance(data, list):
        items = data
    elif isinstance(data, dict):
        # try common keys
        for key in ("data", "results", "trades", "items"):
            if isinstance(data.get(key), list):
                items = data[key]
                break
    if not isinstance(items, list):
        print("Unexpected response shape; expected list of trades.")
        return 4

    state = load_state()
    last_key = state.get("last_key")  # could be timestamp or id
    seen_new = False
    now_iso = datetime.now(timezone.utc).isoformat()
    new_log_lines: List[str] = []

    # Build a sortable key: prefer filed date then transaction date
    def key_for(i: Dict[str, Any]) -> str:
        ni = normalize_item(i)
        # combine date + ticker + name as a uniqueness heuristic
        filed = parse_dt(ni.get("filed_date")) or parse_dt(ni.get("transaction_date")) or datetime(1970,1,1,tzinfo=timezone.utc)
        uniq = f"{ni.get('name','')}|{ni.get('ticker','')}|{ni.get('type','')}|{ni.get('amount','')}|{ni.get('url','')}"
        return f"{filed.isoformat()}::{uniq}"

    items_sorted = sorted(items, key=lambda i: key_for(i))

    # Process and filter
    for raw in items_sorted:
        ni = normalize_item(raw)
        if not is_pelosi(ni.get("name"), names):
            continue
        k = key_for(raw)
        if last_key and k <= last_key:
            continue
        # New item
        seen_new = True
        tdt = parse_dt(ni.get("transaction_date"))
        fdt = parse_dt(ni.get("filed_date"))
        tdt_s = tdt.isoformat() if tdt else (ni.get("transaction_date") or "?")
        fdt_s = fdt.isoformat() if fdt else (ni.get("filed_date") or "?")
        line = (
            f"- [{now_iso}] {ni.get('name')} filed={fdt_s} trade={tdt_s} "
            f"{ni.get('type','?')} {ni.get('ticker','?')} amount={ni.get('amount','?')} "
            f"{('('+ni.get('url')+')') if ni.get('url') else ''}"
        )
        new_log_lines.append(line)
        last_key = k

    if seen_new:
        append_log(["\n## New items", *new_log_lines])
        save_state({"last_key": last_key})
        print(f"Logged {len(new_log_lines)} new Pelosi trade(s).")
    else:
        print("No new Pelosi trades.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
