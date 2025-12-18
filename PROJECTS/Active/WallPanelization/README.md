# WallPanelization (Project Hub)

This folder links planning artifacts in APM with the WallPanelization implementation repo.

Contents
- repo/ — Git submodule to https://github.com/jmpz63/WallPanelization (JSON contract, GhPython exporter, CI)
- Components/ — subsystem catalogs and templates (specs, interfaces, power, risks, BOM)
- Power/ — power budget, generator sizing notes, single-line/layout placeholders, compute_power.py
- Software/ — structure notes and a synthetic JSON generator (no Rhino required)

Manufacturing Cell Specs
- Components/Manufacturing_Cell/ — hybrid rotatable table workcell specs and panel recipe schema
	- Workcell_Spec.md
	- Rotatable_Table_Spec.md
	- panel_recipe.schema.json

Quick links
- JSON contract (in submodule): repo/docs/JSON_CONTRACT.md
- Schema: repo/schema/fabrication.schema.json
- Samples: repo/samples/
- Working repo (local): C:\Users\jmpz6\OneDrive\Moveo\WallPanelization
- Working repo (remote): https://github.com/jmpz63/WallPanelization

Roles
- APM: knowledge base, planning, and high-level management (this folder).
- WallPanelization repo: day-to-day development and artifacts; this APM folder references it as a submodule under `repo/`.

Generate synthetic data (no Rhino)
- python Software/synthetic_generator.py

Next steps
- Fill in Components/* specs per subsystem
- Populate Power/power_budget.csv with vendor data as it’s chosen
- Run Power/compute_power.py to roll up loads and estimate generator size
