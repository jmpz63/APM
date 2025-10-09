#!/usr/bin/env python3
"""
Generate synthetic panel JSON compatible with repo/schema/fabrication.schema.json
No external dependencies. Use to unblock ROS work before Grasshopper is ready.
"""
import json
from pathlib import Path

OUT = Path(__file__).resolve().parents[2] / "repo" / "samples" / "synthetic_panel.json"


def studs_along(length_mm, spacing_mm, stud_len_mm, width_mm, depth_mm, panel_id):
    comps = []
    x = 0.0
    idx = 0
    while x <= length_mm:
        comps.append({
            "id": f"STUD_2X4_{idx:03d}",
            "type": "STUD_2X4",
            "panel_id": panel_id,
            "pos_x_mm": round(x, 3),
            "pos_y_mm": 0.0,
            "pos_z_mm": 0.0,
            "length_mm": stud_len_mm,
            "width_mm": width_mm,
            "depth_mm": depth_mm
        })
        x += spacing_mm
        idx += 1
    return comps


def sheathing_sheet(cx_mm, cy_mm, thickness_mm, len_mm, width_mm, panel_id, idx):
    return {
        "id": f"SHEATHING_OSB_{idx:03d}",
        "type": "SHEATHING_OSB",
        "panel_id": panel_id,
        "pos_x_mm": cx_mm,
        "pos_y_mm": cy_mm,
        "pos_z_mm": thickness_mm / 2.0,
        "length_mm": len_mm,
        "width_mm": width_mm,
        "depth_mm": thickness_mm
    }


def main():
    panel_id = "PANEL_SYN_1"
    wall_len = 3658.0
    stud_spacing = 406.4  # 16 in in mm
    stud_len = wall_len
    stud_w = 38.0
    stud_d = 19.0

    data = studs_along(wall_len, stud_spacing, stud_len, stud_w, stud_d, panel_id)
    data.append(sheathing_sheet(1219.0, 0.0, 3.0, 2438.0, 1219.0, panel_id, len(data)))

    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    raise SystemExit(main())
