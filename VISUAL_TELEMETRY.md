> Current release/deployment status: [README.md](README.md) and [DEPLOYMENT.md](DEPLOYMENT.md). Earlier dated status statements below are historical.

> Historical release note. Deployment IDs, state snapshots and validation counts below describe that release. Current source behavior and deployment status are in [README.md](README.md) and [SEARCH_RECOVERY.md](SEARCH_RECOVERY.md).

# Live map previews and detection telemetry

A first valid, capture-time map observation is now drawn immediately, including moving, off-centre and high-uncertainty observations. The marker is labelled Preview and never changes the target-estimator likelihood, mission confirmation, navigation or safety decisions. A retained coordinate fades and is labelled Last seen after loss; map, initial-pose, stage and mission changes clear the preview. A fresh fused estimate takes priority, and an accepted target takes priority over both.

When capture-time map bearing and a valid pixel range exist but covariance validation blocks the mission measurement, the UI may show a coarse point with at least 15 cm uncertainty. It uses the image-time ray, never the latest robot pose. Without a capture-time map transform, the panel explains why no point can be drawn. Beyond the selected maximum, it shows a direction only, without extrapolating distance. An off-map preview has an explicit edge label while retaining its actual coordinates.

The camera panel now displays map position and uncertainty, group progress, YOLO confidence, raw/in-range/usable counts, distance and range type, original AABB pixels, centring offset, motion means and span, posture stability, image age, inference time, robot position uncertainty and the current blocking reason. Unknown measurements remain unavailable rather than becoming zero.

`VisualTelemetry` is read-only state inside the Console; `/project/target_estimate` and all mission/fusion modules are unchanged. Tests cover first-frame display, stale retention, invalid inputs, missing transform, out-of-range direction, target association, desktop/mobile layout and unchanged confirmation behavior. Local validation: 847 Python tests and 68 browser tests passed. Native robot checks and deployment readback are recorded under `validation/` when complete.
