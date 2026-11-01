# EE3033 — map, find and return

ROS 2 Humble software for on-site mapping, autonomous floor-target search,
map-frame position reporting and verified return to the recorded start.

## Deployed release

`merged_single_frame_wall_tip_20261101_513edd1751` combines the single-frame
bottle workflow with wall-tip navigation and narrow-opening fixes. The 93
production source files match the robot deployment recorded in
[release-manifest.json](release-manifest.json).

- Accept one valid timestamped bottle exposure with its actual uncertainty;
  allow useful exposures during motion and at most one bounded centering attempt.
- Record detection, coordinate reporting and return outcomes separately.
- Prefer a 3 cm route-tracking reserve, retaining full-footprint narrow connectors.
- Use 12 cm lookahead and the validated terminal path tangent for automatic mapping.
- Bound scan matching around odometry in repeated-wall environments; retain loop closure.
- Use a bounded whole-route return budget and a 6 cm inner home target while
  retaining independent arrival and stationary verification.
- Keep every operator-facing UI message in English.

The measured chassis remains 11.5 × 18 cm, with a conservative physical radius
of 0.106801 m and padded verification radius of 0.116801 m. SLAM uses 5 cm cells;
local costmaps and refined global navigation grids are separate resolutions.

## Build and operate

Use the existing robot drivers and verified camera calibration. Build with
`scripts/build_robot.sh`, then source the resulting overlay. Launch
`ee3033_project assessment.launch.py` and open the operator console at
`http://127.0.0.1:8765/#/workflow` through an authenticated localhost SSH forward.
See [deployment](DEPLOYMENT.md), [agent instructions](agent.md), and
[bottle localization](BOTTLE_LOCALIZATION.md) before operating hardware.

Loading a stage prepares its sensors and waits for operator Start. Keep the
target absent during mapping, review the new map, establish the assigned start
pose, and then start autonomous search. Stop cancels owned work and closes motion.

## Verification

987 Python regressions and 37 focused native ROS cases passed. The unchanged
simulation field completed mapping, search/report and verified return. A
non-bypassable 34 cm doorway passed bidirectional simulated motion; six free
5 cm cells (30 cm) passed native planning with the unchanged footprint.

These are software/simulation results. The physical robot deployment and runtime
parameters were read back, but no completed physical end-to-end acceptance is
claimed here. Single-frame coordinates retain approximate quality and uncertainty.
See [TESTING.md](TESTING.md) and [validation summary](validation/release-validation.json).

No new live maps, telemetry, credentials, private deployment backups or generated
runtime logs are included in this release. Existing versioned historical evidence
and fixed regression fixtures remain available.
