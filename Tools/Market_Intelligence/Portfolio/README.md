# Portfolio Dashboard

This folder holds a manual positions file and scripts to generate a living portfolio snapshot with live quotes and option payoff scenarios.

- `positions.json`: Edit this to reflect your current positions (stocks, crypto, options). Pre-populated from your recent activity.
- `update_portfolio_snapshot.py`: Fetches live prices (stocks via Stooq; BTC via CoinGecko) and renders `portfolio_snapshot.md`.
- `options_helper.py`: Builds payoff tables for listed options using your premiums and expirations.

Usage
```powershell
cd "$Env:OneDrive\APM\Tools\Market_Intelligence\Portfolio"
python update_portfolio_snapshot.py
python options_helper.py
```

Notes
- No credentials required. Stock quotes use Stooq CSV; BTC uses CoinGecko.
- If a symbol fails to fetch, it will be noted and retained with last known cost only.
- You can add API keys later for richer data if desired.