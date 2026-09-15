# Initial mechanism concepts and evaluation contract

Date: 2026-09-14. Phase 0 proposal only. No CAD, mechanism implementation, simulations, purchasing, or physical experiments were performed. Every proposed dimension and score below is **ESTIMATED**, chosen to define a comparison rather than establish agronomic suitability. No vendor price or physical performance claim is made.

## Existing mechanical evidence

- `src/weedbot_simulation_tertill/urdf/weedbot.urdf`: cylindrical base radius 105 mm, four wheels radius 35 mm, wheel separation 140 mm, front bumper and belly contact disc. No operating cutter geometry. File contains merge markers. Wheel joint axes and sensor heights need reconciliation with their visuals and transforms before using clearance results.
- `deweeder_tertill/models/weedbot/model.sdf`: different chassis (350 x 300 x 120 mm), two 80 mm radius wheels, 220 mm track; a fixed **visual-only** trimmer box of 160 x 60 x 40 mm. This is not a validated tool model or shared hardware specification.
- `src/deweeder_robot/urdf/deweeder_robot.urdf`: 500 x 300 x 150 mm chassis with fixed track links, shoulder/elbow arm, spherical end effector. Preserve as historical simulation; freeze arm-led mechanism development.
- `worlds/Farm of Deweeder.wbt`: 2 m square visual field, five 100 mm square crop representations at 300 mm spacing, three red spherical weeds of radius 60 mm, E-puck and separately declared arm robot. Geometry is demonstrative, not crop measurements.
- No STEP, STL, CadQuery, FreeCAD, OpenSCAD, manufacturing drawings, tool BOM, or physical mechanism measurements were found in the inspected working tree. Two reference PDFs exist; their contents were not reviewed for this concept proposal and no claims are attributed to them.
- `ToolCmd.msg` has `front_trimmer_on`, `belly_trimmer_mode`, `agitator_pulse`; retain compatibility while defining a future bench adapter. The GPIO HAL optionally calls a driver `apply_tool`, absent from the declared motor driver protocol. The symbolic pin map names four mobility motors and no actual tool wiring. Tool status, coast-down and current measurement availability remain unverified.

## Three concepts for one interchangeable trolley mount

All use manual height/depth adjustment, one driven tool axis, and an externally moved rail/trolley. Passive compliance does not count as an active DOF. Moving-part counts are approximate major external moving members, excluding motor internals, bearings and fasteners. Common bench hardware is excluded from relative tool cost.

| Attribute | A: guarded rotating line cutter | B: shallow rotating tine cartridge | C: reciprocating mini-hoe |
|---|---|---|---|
| Principle | Horizontal rotating replaceable line cuts above soil; fixed guard restricts accessible area | Small vertical spindle with replaceable bent tines disturbs shallow soil | Motor eccentric drives a guided blade back and forth at shallow depth |
| Driven DOF / moving members | 1 / about 3 (hub plus two line ends) | 1 / about 3-5 (hub plus 2-4 tines) | 1 / about 4 (crank, rod, slider, blade) |
| Actuation | One speed-controlled motor; spin-down measured | One geared motor; torque/current limit | One geared motor, eccentric and guarded slider |
| Initial comparison width | 80 mm diameter; sweep 40-120 mm | 80 mm diameter; sweep 40-120 mm | 60 mm blade plus 20 mm peak-to-peak lateral stroke; sweep total 40-120 mm |
| Initial vertical setting | 5-20 mm above nominal soil | 5-15 mm below nominal soil | 5-15 mm below nominal soil |
| Crop exclusion | Swept line plus deflection uncertainty and guard/body envelope | Tine swept area plus root-zone uncertainty and guard envelope | Full swept blade/stroke plus backlash uncertainty and guard envelope |
| Main wear / replacements | Line, spool, bearings; replace line/spool | Tine tips, shaft seals, bearings; replace bolted tines | Blade edge, rod ends, guide; replace blade/bushes |
| Approximate BOM | Motor/controller, hub, line, shield, bearing support, bracket | Gearmotor/controller, supported shaft, hub, tines, shield, skid, bracket | Gearmotor/controller, eccentric, rod, slider/guide, blade, shield, skid, bracket |
| Relative acquisition cost | Lower, uncertain | Medium, uncertain | Medium, uncertain |
| Manufacture | Simple bracket/shield; balanced hub required | Cut/bend tines, shaft alignment and robust hub | Cut blade and plate; accurate guide/alignment required |
| Clogging | Vegetation can wrap around hub | Roots/residue can wind around spindle | Residue can jam blade/guide |
| Stone/root sensitivity | Line breakage and thrown debris | Impact loading, tine deformation or stall | Blade chipping, guide overload or stall |
| Mud sensitivity | Shield accumulation, reduced clearance | Accumulation and depth/torque changes | Abrasive contamination of guide and blade |
| Maintenance complexity | Lower, frequent consumable changes possible | Medium, inspection and tine changes | Medium-high, alignment and lubrication |
| Travel-speed hypothesis | Test 0.1-0.5 m/s; no proven operating limit | Test 0.1-0.5 m/s; load may limit | Test 0.1-0.5 m/s; stroke coverage may limit |

Guard design is an unvalidated protection hypothesis. A guard touching a crop can itself cause damage; its contact must be logged separately. Root damage can occur without stem contact. Motor-off does not instantly eliminate rotating or oscillating hazard. No concept currently justifies timed cutting immediately beside crop stems.

## Preliminary weighted decision matrix

Scores are engineering hypotheses on 1 (unfavourable) to 5 (favourable). Higher cost score means cheaper. These are not simulation outputs, purchase quotes or test results. Weights retain the user's suggested priorities. Precision scores reflect the expected controllability of the envelope; safety includes physical hazards beyond geometric stem overlap.

| Criterion | Weight | A | B | C |
|---|---:|---:|---:|---:|
| Precision | 25% | 2 | 3 | 4 |
| Crop safety | 25% | 2 | 2 | 3 |
| Cost | 15% | 4 | 3 | 3 |
| Reliability | 10% | 3 | 3 | 2 |
| Serviceability | 10% | 5 | 4 | 3 |
| Speed | 10% | 4 | 3 | 3 |
| Energy | 5% | 3 | 2 | 3 |
| Weighted score / 5 | 100% | **2.90** | **2.80** | **3.15** |

A receives low precision for flexible line deflection; B receives low safety for below-soil intrusion; C trades a more bounded blade envelope for additional linkage wear. Energy and speed scores are particularly weak assumptions. Allow at least +/-1 score uncertainty per criterion (bounded 1-5). A one-point change in precision alone equals 0.25 overall, as large as C's lead over A; this ranking cannot select a winner. Compare all three at equal swept width, path and field. Start with fixed inter-row travel and an inactive dummy envelope. Add crop-synchronised actuation only if fixed geometry fails the crop-clearance/coverage trade-off.

## Minimum geometry experiment

Use a lightweight planar simulator before integrating Webots or ROS. Define +x along row, +y across row, +z above soil, metres/seconds/radians internally. Height and depth are metadata and geometric constraints, not soil physics. A common 2.5 m observation row has run-in/run-out beyond both ends so actuator start/stop does not bias results.

Required inputs: row spacing; plant spacing and variance; crop radius/height; weed density/distribution/radius; lateral path error; heading error; tool width/depth/active radius/response time; vehicle speed. Add crop-position uncertainty, mechanism envelope/phase, offset, guard footprint, delay jitter, deactivation/coast-down time and seed. State whether variance means variance or standard deviation, use unit-bearing names, and define error distributions and their correlation along a pass. These are synthetic inputs until measured.

For a first common fixture, use 0.50 m row spacing, 0.30 m nominal plant spacing, 0.025 m crop radius and 0.08 m swept tool width; these are invented bench dimensions, not tomato/chilli/onion recommendations. Include both inter-row and offset near-row paths so an easy inter-row result cannot imply intra-row capability.

Represent A/B initially by a conservative full swept disk and C by the full blade/stroke rectangle. This compares accessible geometry only: A and B can produce identical geometric results despite different biological actions. A subsequent phase-resolved envelope can estimate contact opportunities and dwell; it still cannot infer cutting success. Include inactive tool body, guard, and coast-down swept envelopes. Do continuous swept intersection or demonstrate timestep convergence; a point sampled at 64 ms travels 64 mm at 1 m/s and can miss a narrow collision.

Sweep speeds [0.1, 0.25, 0.5, 0.75, 1.0] m/s, lateral errors [0, 5, 10, 20, 30, 50] mm and additional crop margins [20, 30, 40, 50, 75] mm. Treat high speeds as stress cases. Run deterministic fixtures first, then paired seeds across mechanisms, with up to 10,000 independent random rows per selected configuration. Report row count and plant count separately; estimate uncertainty over rows to avoid treating correlated plants as independent. No experiment has yet been run.

Log run/seed/configuration, timestamp, tool and vehicle pose, closest signed crop/weed clearance, command state, actual active state, target ID, exposure event, guard contact and crop-envelope intersection. Deduplicate plants for plant rates; retain separate event IDs for event rates. Export raw events CSV, per-run CSV and metadata/aggregate JSON. Plot common axes with crops, weeds, tool/guard footprints, protected zones, exposures and misses. Use heatmaps labelled **SIMULATED crop-envelope intersection probability** and **SIMULATED weed exposure fraction**, never crop damage or weed removal efficiency.

## Evaluation definitions and gates

| Metric | Definition and evidence boundary |
|---|---|
| Geometric exposure | Unique target weeds intersecting active envelope / all predefined target weeds; SIMULATED proxy only |
| Crop-envelope intersection | Unique crops intersecting hazardous tool envelope / encountered crops; SIMULATED proxy, not damage |
| WRE | Successfully treated unique weeds / total predefined target weeds; success and assessment time preregistered, BENCH-MEASURED or FIELD-MEASURED |
| CDR | Unique damaged crop plants / crop plants encountered; damage rubric and delayed assessment required |
| MR | Untreated target weeds / total target weeds; equals 1-WRE only for exhaustive binary outcomes at the same assessment time |
| FTR | Non-target/crop treatment events / all treatment events; define one event and classify mixed events consistently, do not substitute timestep counts |
| Throughput | 60 x distance_m / elapsed_seconds in m/min; distinguish moving time from full elapsed operation |
| Area capacity | 0.36 x speed_m_per_s x credited_width_m in ha/h theoretical; multiply by measured field efficiency for effective capacity. Acre/h = ha/h / 0.404686 |
| Clearance | Minimum signed crop-surface-to-hazard clearance in geometry; minimum safe weed-to-crop separation requires physical damage evidence and a confidence bound below the agreed threshold |
| Energy | Integral(V x I dt)/3600/distance_m in Wh/m; report tool and mobility separately, subtract no idle energy without an explicit protocol |
| Cost | Labour + energy + consumables + maintenance + amortisation, divided by treated distance/area; ESTIMATED until measured and priced |

Zero denominators yield unavailable, not zero. No weed is excluded from the target denominator after seeing a failure. Report protected-zone weeds separately while retaining them in the declared all-target result. Repeated cutting needs baseline weed size/species, pass interval, regrowth and follow-up observations; an immediate severed stem is not proof of suppression. >90% treatment, eventual >95%, and <2% crop damage remain hypotheses; commercial speed and acceptable cost remain undefined requirements.

QA must cover missing/close crops, overlapping crop/weed disks, high density, impossible inputs, depth/height limits, phase/latency/coast-down, drift and heading error. Sensor loss, motor stall, encoder error, emergency stop, tilt, mobility stopped while tool active, tool timeout and communication loss require separate control/safety fault tests. Geometry alone cannot pass those cases or certify hardware safety.

First decision: can a common fixed tool width retain useful target exposure under +/-30 mm bounded lateral error while respecting the chosen crop exclusion envelope? Reject incompatible geometry before CAD. Next implementation after audit completion: a pure-Python configuration validator, deterministic swept-envelope fixtures and event schema; then seeded Monte Carlo. Physical suppression, power, wear, clogging and root damage remain bench/field questions.
