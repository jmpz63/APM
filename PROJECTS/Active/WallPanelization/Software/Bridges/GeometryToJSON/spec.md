# Bridge: Geometry → JSON Exporter [EXISTS]

Purpose: Convert Grasshopper geometry (or synthetic generator) to fabrication JSON for ROS 2.

Inputs
- Grasshopper solids list (geometry_in) OR synthetic generator params

Outputs
- JSON array matching schema/fabrication.schema.json

SLA
- Deterministic output, millimeter units, consistent IDs
- Run time: O(n) over components (<100 ms for small panels)

Implementation
- GhPython script in submodule repo/ghpython_exporter.py
- Synthetic path in Software/synthetic_generator.py
