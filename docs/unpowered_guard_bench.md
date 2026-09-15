# Unpowered guide/guard bench — specification draft

2026-09-15. **ESTIMATED** dimensions and design choices. This is a reviewable specification, not manufactured hardware, tested protection, or a powered-test procedure.

The multi-pass pilot exposed approximately 70% of targets without observed crop intersections in the inset-four layout. Miss classification now separates near-crop targets, gaps left by nominal fixed offsets, and exposure lost relative to ideal passes. The first physical fixture should measure positioning repeatability and contact geometry before a motor or cutting element is introduced.

## Fixture

Use a straight manually moved carriage with a rigid dummy 80 mm tool disk, interchangeable guard mock-ups and a lateral mounting scale. Observe a 2.5 m row with 0.2 m run-in and run-out; provide at least 2.9 m usable travel. Two lines of replaceable surrogate crop posts are 0.5 m apart. Post radius is 25 mm and nominal spacing is 0.3 m, matching the synthetic experiment. These are fixture dimensions, not crop recommendations.

The carriage guide establishes the reference path. An adjustable mount deliberately introduces offsets of −30, −10, 0, +10 and +30 mm. Mark the four nominal pass offsets 120, 210, 290 and 380 mm from the first crop row. Measure actual positions at the start, middle and end of travel rather than assuming the mounting scale is accurate.

Compare the existing 60 mm radius disk body mock-up with the parameterized U-frame: 140 × 8 mm side rails at y=±50 mm and an 8 × 108 mm rear bar at x=−74 mm. Give each mock-up smooth contact surfaces; record material, thickness and mounting stiffness. Their horizontal projections alone do not specify vertical clearance, debris shielding or real crop compliance. Before CAD fabrication details are finalized, measure the chosen carriage mount, available vertical adjustment and required guard thickness; do not invent fit dimensions from the simulator.

## Minimum parts and measurements

| Item | Purpose / unresolved detail |
|---|---|
| Straight reference guide, manual carriage and rigid base | Establish a repeatable path; actual straightness and play must be measured |
| Laterally adjustable dummy-tool mount and scale | Reproduce pass offsets and imposed errors; attachment dimensions remain open |
| Rigid 80 mm dummy disk | Mark the active envelope without a blade, string, spindle or motor |
| Two interchangeable guard mock-ups | Compare disk and U projections; no passive-protection claim from shape alone |
| Replaceable crop surrogates and removable target markers | Known geometry, including missing posts and close pairs |
| Ruler/caliper, fixed camera view or contact witness marks | Record actual spacing, carriage play, post/guard contact and mount slip |
| Trial sheet / CSV | Store dimensions, setup revision, actual path and observations for every trial |

No component price or India availability has been verified. Obtain dimensions and supplier quotes before producing a priced BOM. This bench does not require ROS2, computer vision, autonomous steering, an encoder or a motor controller to answer the initial geometry/repeatability question.

## Procedure and acceptance gate

1. Record as-built post coordinates, radii, dummy disk radius and every guard dimension. Attach a fixture/configuration revision to the trial record.
2. Move the empty carriage along the guide and measure lateral play and straightness. Log offsets at start/middle/end before installing crop surrogates.
3. Install the dummy disk and each guard in turn. Run all four pass offsets with each imposed lateral offset, ten repetitions per combination. Treat the 400 traversals across two guard profiles as repeatability trials, not 400 independent crop populations.
4. Record every dummy-tool/post and guard/post contact, including contacts when the nominal active envelope would be off. Repeat a selected subset with a missing post, close post pair, and target markers between posts.
5. Compare observed contacts against an export regenerated from **measured** fixture geometry. If contact predictions disagree, inspect alignment, dimensions, vertical effects and compliance before adding complexity.
6. Report achieved lateral repeatability and its distribution. The ±30 mm simulation value is an imposed stress scenario, not a measured specification. Do not promote the fixture to powered use merely because no contact was seen in a small trial set.

For this fixture the proposed gate is no dummy-tool contact with surrogates in the inset-four nominal cases, documented guard contacts, repeatable mount position, and an explanation for every simulation/fixture discrepancy. This is a geometry gate only. It does not establish crop injury tolerance, weed suppression, impact containment or fail-safe stopping.

The fixture must contain **no motor, cutting line or sharp active blade**. Powered work requires the separately outstanding HAL/ROS emergency-stop, timeout, readiness and tool-control repairs, plus a physical power-isolation design and measured coast-down. Do not repurpose this unpowered procedure as a powered one.

Parameter source: [unpowered_guard_bench.yaml](../configs/unpowered_guard_bench.yaml). Next CAD work should model the measured carriage interface and interchangeable mock-ups parametrically, export STEP/STL, and keep mounting dimensions explicitly unverified until measured. No manufacturing or purchasing was performed.
