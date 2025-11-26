# Vibratory Feeder Drive Selection Project Hub

## Roles & Scope
- **APM Hub (this README)**: Centralized documentation, decision criteria, test results, and cross-links.
- **Working Assets**: CAD, vendor datasheets, test logs (external folders or submodules when added).
- **AI Assistants**: Maintain `How_to_Choose_Vibratory_Drive.md`, add empirical data, ensure compliance after updates.

## Objectives
1. Characterize target parts (size, mass, material friction).
2. Determine feed rate targets (steady-state + peak).
3. Select candidate base (AC vs DC; standard vs Fast-Angle).
4. Prototype orientation track sections; measure success rate.
5. Optimize spring tuning for stable throughput and minimal part bounce.
6. Establish acceptance tests (rate stability, orientation reliability, thermal performance).

## Cross-Links
- Engineering Knowledge: `Engineering/Vibratory_Feeding/How_to_Choose_Vibratory_Drive.md`
- Templates: (Add future testing & data collection templates under `Tools/Templates/Feeding/`)

## Initial Data Required
| Data | Status |
|------|--------|
| Part dimension spectrum | Pending |
| Average part mass | Pending |
| Desired throughput (parts/min) | Pending |
| Orientation complexity (simple/moderate/complex) | Pending |
| Trial video sample | Pending |
| Candidate vendor models list | Pending |

## Decision Criteria (Snapshot)
| Driver Type | Use Case | Notes |
|-------------|----------|-------|
| AC 7200 VPM | Small/light parts; high rate | Higher frequency, shorter stroke |
| DC 3600 VPM | Larger/heavier parts | Gentler handling, longer stroke |
| Fast-Angle Variant | Need more linear motion | Reduced vertical component |

## Test Plan (Draft)
1. Baseline feed test at nominal amplitude, 60% bowl fill.
2. Vary fill (40%, 80%) record feed rate & orientation accuracy.
3. Introduce orientation tooling sections sequentially; log success/failure.
4. Thermal monitoring of drive coil over 30 min continuous run.
5. Vibration spectrum capture (accelerometer) for resonance identification.

## Metrics to Track
- Feed rate (parts/min) vs amplitude & fill.
- Orientation success % per station.
- Coil temperature rise (°C) over time.
- Power consumption.
- Vibration frequency stability.

## Risks & Mitigations
| Risk | Mitigation |
|------|-----------|
| Overloading base unit | Lightweight bowl / reduce fill / proper sizing |
| Part bounce misorientation | Fast-Angle variant / reduce amplitude |
| Throughput instability | Tune springs / maintain optimal fill window |
| Thermal drift | Monitor temp / add cooling interval if needed |

## Workflow Compliance
After updating data or adding logs:
```python
from _ADMIN.ai_agent_interface import notify_new_document, ensure_compliance
notify_new_document("PROJECTS/Active/vibratory_feeder_selection/README.md","Vibratory feeder hub update","medium")
ensure_compliance()
```

## Roadmap
| Phase | Focus | Outcome |
|-------|-------|---------|
| 1 | Data collection | Part profile + initial candidate list |
| 2 | Prototype tests | Empirical feed & orientation metrics |
| 3 | Optimization | Spring tune + tooling refinement |
| 4 | Selection | Final drive spec + vendor docs |
| 5 | Documentation | Completed hub + integration into master index |

## Metadata
Status: Initialized (2025-11-23)
Next Action: Collect part dimension & mass dataset.
