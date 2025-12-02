# Political Trades Monitor

This tool polls a data provider for Congressional trades and alerts when new filings for Pelosi (Nancy/Paul) appear.

Important: There is no true real-time for Congressional trades. The STOCK Act requires reports days/weeks after trades. This monitor alerts within minutes of a filing being posted to your chosen provider.

## Setup

1) Choose a provider (examples):
- Quiver Quantitative API (recommended; requires API key)
- Capitol Trades, Unusual Whales, or other aggregators with JSON endpoints
- Official portals (House Clerk, Senate eFD) are harder to automate; most require scraping/logins.

2) Configure provider:
- Copy `provider_config.example.json` to `provider_config.json` and fill in:
```json
{
  "provider_url": "https://api.quiverquant.com/beta/live/congresstrading",
  "headers": { "Authorization": "Bearer YOUR_QUIVER_API_KEY" },
  "names": ["Nancy Pelosi", "Paul Pelosi"]
}
```
- Alternatively, set environment variables:
  - `PROVIDER_URL` (string)
  - `PROVIDER_HEADERS_JSON` (JSON string, e.g., {"Authorization":"Bearer KEY"})
  - `POLITICIAN_NAMES` (comma-separated)

## Run once
```powershell
C:/Users/jmpz6/AppData/Local/Microsoft/WindowsApps/python3.11.exe "c:\Users\jmpz6\OneDrive\APM\Tools\Market_Intelligence\Political_Trades\monitor_pelosi_trades.py"
```
Outputs:
- `politician_trades.md`: appended log of new items
- `monitor_state.json`: remembers last seen filing to avoid duplicates

## Schedule (Windows)
Use `schedule_politician_monitor.ps1` to run every 15 minutes.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "c:\Users\jmpz6\OneDrive\APM\Tools\Market_Intelligence\Political_Trades\schedule_politician_monitor.ps1" -Minutes 15
```

To query the task after creation:
```powershell
schtasks /Query /TN APM_Pelosi_Trade_Monitor
```
