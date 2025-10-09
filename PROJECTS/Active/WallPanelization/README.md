# WallPanelization (Project Hub)

This folder links planning artifacts in APM with the WallPanelization implementation repo.

Contents
- repo/ — Git submodule to https://github.com/jmpz63/WallPanelization (JSON contract, GhPython exporter, CI)
- Components/ — subsystem catalogs and templates (specs, interfaces, power, risks, BOM)
- Power/ — power budget, generator sizing notes, single-line/layout placeholders, compute_power.py

Quick links
- JSON contract (in submodule): repo/docs/JSON_CONTRACT.md
- Schema: repo/schema/fabrication.schema.json
- Samples: repo/samples/

Next steps
- Fill in Components/* specs per subsystem
- Populate Power/power_budget.csv with vendor data as it’s chosen
- Run Power/compute_power.py to roll up loads and estimate generator size
