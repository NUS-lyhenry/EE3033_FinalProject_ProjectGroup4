> Current release/deployment status: [README.md](README.md) and [DEPLOYMENT.md](DEPLOYMENT.md). Earlier dated status statements below are historical.

# Examination runbook and evidence

The local `single_frame_simplification_20261101` source update is not deployed
or physically validated. Earlier deployment receipts describe the prior version.
Read [SEARCH_RECOVERY.md](SEARCH_RECOVERY.md) before testing its new behavior.

Use the existing Humble container and separate project overlay. Open the console
at `http://127.0.0.1:8765` through the existing SSH tunnel.

## Preparation

- Verify the real footprint, lidar self-return geometry, camera optical axes,
  image/CameraInfo resolution and timestamped TF. Check measured battery voltage.
- Use a floor target with a visible base. Check coordinates at independently
  measured distances/bearings. Bottle mode uses the fixed-mount 17-point model;
  validate image geometry/model compatibility and the selected detection range.
  Physical mounting/reference forms are removed. For explicit non-bottle legacy
  geometry, pass a reviewed configuration file and reload the mission.
- Start with Original speed. Fast trial is 0.16 m/s, 0.60 rad/s; physical turn, stopping and localization checks must precede its
  adoption as the default. Apply speed/clearance only while stopped.
- Keep exactly one final velocity writer and scan adapter. Do not run competing
  SLAM/AMCL stacks. No driver, PID or original map is modified.
- Confirm examiner intervention rules and return tolerance. Record the deployed
  commit, map path and independent reference measurements.

## Single-page trial

1. Launch `assessment.launch.py` with persistent map/result directories.
   **Start automatic mapping** loads services if needed and begins exploration.
   The combined timer starts when mapping services load.
2. Keep the target absent. Automatic mapping uses the existing frontier/retry
   algorithm, returns and saves a unique map. Stop and save explicitly if needed.
   Inspect the map; partial maps remain labelled partial.
3. Select the actual model target class and return tolerance, then **Confirm map
   & prepare mission**. This records review, shuts down the owned mapping stack,
   and loads saved-map localization and perception. It does not start stage 2.
4. After target placement, establish the assigned robot pose in the map. Wait
   for readiness, then **Start autonomous mission**. The mission handles search,
   improving an uncertain observation, one target report and autonomous return.
   The 0.10 m target sigma remains a soft precision objective. A valid bottle
   coordinate is accepted immediately with its actual sigma and source age;
   the 90-second acquisition upper bound is not an extra reporting wait. Bottle
   results use one real valid exposure, allowing ordinary motion. Accepted coordinates
   are retained while at most one useful centering turn runs (up to 2.5 seconds,
   preferred ±32 px normalized to 640 px width). `target_reported` distinguishes
   acceptance from final reporting. Inspect the quality, source age, sigma,
   recovery reason and search completeness in the result and exported JSON.
5. Completion requires return within tolerance and a fresh 0.5-second XY hold
   (span at most 0.05 m, odometry gap/age at most 0.25 s), with fresh zero output commands. Read results and
   download trial JSON. NOT_FOUND, FAILED and unverified navigation success do
   not count as completion. The result panel separates detection, accepted or
   reported coordinates, and return verification. STOPPED does not erase an
   accepted coordinate or prove that a preview was reported.
6. Inform the examiner before intervention/restart. Nothing auto-restarts.
   **重新建图** in the map header preserves earlier maps/results and loads fresh mapping.
   Same-boot console restarts preserve timing; reboot marks it interrupted.

Advanced contains optional manual/waypoint mapping, map selection, speed,
clearance, recovery controls and diagnostics. Camera mounting controls are
only available for the explicitly simulated ROS camera. Stage 2 disallows manual driving, selected navigation
goals, map edits and target-coordinate entry. Stop remains available during
navigation, acquisition, localization and stage/parameter requests.

## Evidence

Record mapping/combined time, target class/confidence, reported/reference XY,
Euclidean error, actual return error/tolerance, position-hold results, battery,
commit/map, interventions and who measured each reference. Keep generated
logs/images/maps outside Git. Camera sigma models uncertainty; stable readings
alone cannot demonstrate absolute accuracy or remove systematic pitch error.

The authoritative local project instructions require a 30-minute demonstration
session including setup and questions. The report is due within 24 hours after
the demonstration, at most 10 pages including cover/appendices and excluding
source code, single column/single spaced, Times New Roman at least 12 pt, with
the repository link. See TESTING for software evidence and remaining physical
validation; no complete physical mission is claimed by software checks.
