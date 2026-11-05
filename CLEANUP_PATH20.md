> Current release/deployment status: [README.md](README.md) and [DEPLOYMENT.md](DEPLOYMENT.md). Earlier dated status statements below are historical.

> Historical release note. Deployment IDs, state snapshots and validation counts below describe that release. Current source behavior and deployment status are in [README.md](README.md) and [SEARCH_RECOVERY.md](SEARCH_RECOVERY.md).

# UI cleanup and 2 cm path checks

Deployed release: `cleanup_path20_20261031_89c7bfadfe`.

- `PATH_SEARCH_SPACING = 0.02` m. Default path samples use half this spacing (0.01 m).
- Physical robot pages no longer create the camera mounting or attitude reference forms, for all target classes. Simulation camera controls are created only for the explicitly simulated ROS camera.
- Retired calibration navigation falls back to the workflow. Detection details and uncertain map previews remain available.
- Existing maps, trial session and safety settings were retained. No movement test was initiated by this deployment.

Validation: 847 local Python tests; 145 focused native ROS Humble tests; native console, visual telemetry and bottom-range integrations. See [archived runtime readback](validation/historical/cleanup-path20-runtime-readback.json) for the module imported using the active mapper process environment. Browser results are recorded separately.
