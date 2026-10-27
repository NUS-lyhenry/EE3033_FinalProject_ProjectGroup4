# Examination runbook and evidence

Use the existing Humble container and separate project overlay. Open the console
at `http://127.0.0.1:8765` through the existing SSH tunnel.

## Preparation

- Verify the real footprint, lidar self-return geometry, camera optical axes,
  image/CameraInfo resolution and timestamped TF. Check measured battery voltage.
- Use a floor target with a visible base. Check camera-only coordinates at
  independently measured distances/bearings. Enter measured mounting values
  under Advanced only when the installed TF is unsuitable; no bottle collection
  is needed. Reload the mission after saving geometry.
- Start with Original speed. Fast trial is 0.16 m/s, 0.60 rad/s with a 0.55 rad/s
  shim turn; physical turn, stopping and localization checks must precede its
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
   The 0.10 m target sigma is a soft precision objective. After bounded
   refinement, a fresh stable estimate may be reported as approximate with its
   actual sigma; inspect that quality label in the result and exported JSON.
5. Completion requires return within tolerance and the existing two-second
   stationary verification with fresh zero output commands. Read results and
   download trial JSON. NOT_FOUND, FAILED and unverified navigation success do
   not count as completion.
6. Inform the examiner before intervention/restart. Nothing auto-restarts.
   **重新建图** in the map header preserves earlier maps/results and loads fresh mapping.
   Same-boot console restarts preserve timing; reboot marks it interrupted.

Advanced contains optional manual/waypoint mapping, map selection, speed,
clearance and camera tools. Stage 2 disallows manual driving, selected navigation
goals, map edits and target-coordinate entry. Stop remains available during
navigation, acquisition, localization and stage/parameter requests.

## Evidence

Record mapping/combined time, target class/confidence, reported/reference XY,
Euclidean error, actual return error/tolerance, stationary results, battery,
commit/map, interventions and who measured each reference. Keep generated
logs/images/maps outside Git. Camera sigma models uncertainty; stable readings
alone cannot demonstrate absolute accuracy or remove systematic pitch error.

The authoritative local project instructions require a 30-minute demonstration
session including setup and questions. The report is due within 24 hours after
the demonstration, at most 10 pages including cover/appendices and excluding
source code, single column/single spaced, Times New Roman at least 12 pt, with
the repository link. See TESTING for software evidence and remaining physical
validation; no complete physical mission is claimed by software checks.
