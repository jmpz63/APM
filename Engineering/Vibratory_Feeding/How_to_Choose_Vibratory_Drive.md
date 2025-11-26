# How to Choose a Vibratory Drive

## Purpose
Guidance for selecting an appropriate vibratory feeder base unit (drive) based on part characteristics, throughput goals, and motion profile requirements.

## Core Concepts
| Aspect | AC Springing | DC Springing |
|--------|--------------|--------------|
| Nominal VPM (60 Hz line) | 7200 | 3600 |
| Stroke Characteristics | Higher frequency, shorter stroke | Lower frequency, longer stroke |
| Typical Use | Small/light parts; higher feed rates | Larger/heavier parts; gentler handling |

VPM = vibrations per minute; changing spring rate or frequency alters part movement dynamics.

## Selection Inputs
1. Part size (max dimension, aspect ratio)
2. Part weight (single + average in-bowl mass)
3. Part material (friction, tendency to bounce, surface sensitivity)
4. Desired feed rate (parts/min or bulk mass flow)
5. Orientation complexity (track length needed for reliable orientation)
6. Available footprint (mounting space constraints)
7. Bowl material & weight (standard vs lightweight fabrication)

## Bowl & Base Interactions
- Lower total bowl weight can allow downsizing the base unit → reduced footprint & cost.
- A larger diameter bowl on a suitable base increases: in-bowl storage, orientation opportunities along a longer track, and buffer capacity.
- Lightweight bowls recommended when marginal on drive capacity or minimizing inertial load for responsiveness.

## Motion Profile Considerations
| Requirement | Recommended Base Variant |
|-------------|--------------------------|
| More linear (horizontal) part travel | Fast-Angle (FA) reduced spring angle design |
| Standard vertical agitation + forward progression | Standard-design feeder |
| Fragile parts needing reduced impact energy | Lower frequency (DC sprung) + tuned amplitude |

Fast-Angle feeders reduce vertical acceleration; this helps with parts that flip undesirably or stall due to excessive bounce. Factory consultation required for exact angle and spring configuration.

## Decision Flow (Simplified)
```text
Start
 ├─ Are parts very small/light (< X grams)? → Favor AC 7200 VPM base
 ├─ Are parts heavy / bulk mass high? → Favor DC 3600 VPM base
 ├─ Is orientation track length insufficient? → Consider larger bowl on same base (check load capacity)
 ├─ Are parts tumbling or bouncing excessively? → Evaluate FA (Fast-Angle) base or reduce frequency
 └─ Space constrained? → Evaluate lightweight bowl or smaller base with bowl weight reduction
```

## Engineering Checklist (Pre-Selection)
- [ ] Measure representative part dimensions & mass (min/avg/max)
- [ ] Quantify required feed rate (steady-state & peak)
- [ ] Classify part orientation steps (simple / moderate / complex)
- [ ] Determine acceptable part impact energy (fragility assessment)
- [ ] Assess bowl fill level at target throughput (avoid overfill stall)
- [ ] Verify base unit dynamic capacity vs bowl + part aggregate mass
- [ ] Confirm mounting footprint allowance & clearance for track exits
- [ ] Evaluate need for linear motion emphasis (FA option)

## Risk & Mitigation
| Risk | Cause | Mitigation |
|------|-------|------------|
| Inconsistent feed rate | Overfilled bowl / improper spring tuning | Maintain optimal fill; retune springs |
| Part bounce / misorientation | Excess vertical acceleration | Fast-Angle base or lower frequency |
| Drive overheating | Excess inertial load from heavy bowl | Lightweight bowl design; correct sizing |
| Orientation failures | Insufficient track length | Upsize bowl diameter; redesign tooling |
| Excess noise/vibration transfer | Poor mounting isolation | Add isolation mounts; verify foundation stiffness |

## Factory Consultation Triggers
Engage vendor when:
- Aggregate bowl + part mass near upper limit of candidate base.
- Motion requires hybrid linear-vertical tuning beyond standard offering.
- Parts have unusual geometry causing persistent flip/stall patterns.
- High-precision feed rate stability needed for downstream automation.

## Integration Notes
- Cross-link with `Projects/Active/vibratory_feeder_selection/` for project-specific calculations.
- Future expansion: add empirical amplitude vs feed rate curves, vibration spectrum logs, and orientation performance metrics.

## Next Data to Capture
1. Part sample video under trial feed conditions.
2. Preliminary bowl fill vs feed rate test at 50%, 70%, 90% fill.
3. Spring tuning record: frequency, amplitude, coil temp over 30 min run.
4. Orientation success rate across track stations.

## Document Metadata
Source: User-provided guidance (2025-11-23)
Status: Initial draft integrated
Revision Path: Add quantitative decision tables & empirical test data.
