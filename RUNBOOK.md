# Examination runbook and evidence

Use the existing Humble container and the separate project overlay. The operator
console is at <http://127.0.0.1:8765> through the existing SSH tunnel. Do not start
a duplicate tunnel or driver. See README for exact launch commands.

## Before the trial

1. Check the physical footprint, lidar self-return geometry, optical transform,
   camera calibration, target surface/height, and battery voltage. Validate the
   coordinate method with independently measured positions. Software defaults
   are not calibration evidence.
2. Check exactly one `/cmd_vel` publisher (the project gate), no competing SLAM/
   AMCL transform, fresh lidar/odometry/TF, and the actual map/save directory.
3. Open **Navigation settings**. Select Cautious, Balanced or Bold while stopped, then
   configure gap blocking; verify global/local
   radii, mapping/search clearance and turn-stop distance. Apply temporary trial limits only in
   idle mapping. Confirm no control fault. Defaults return after restart.
4. Have the examiner confirm intervention rules and return position tolerance.
   Record the deployed local commit and map path in TESTING's trial table.

## Mapping, then mission

1. Launch `assessment.launch.py` with the persistent map/results directories and
   `allow_motion:=true` only for the prepared robot. Open **Mapping** and **Load
   mapping**. The combined timer starts as the mapping stack loads; the UI's
   separate stage timer is still shown. Keep an examiner stopwatch as reference.
2. Keep the target absent during mapping. Start automatic mapping, or use manual
   mapping only as permitted. Save in a new directory. Inspect and mark the map
   reviewed; `MAPPED_PARTIAL` remains partial after review.
   For a mapping waypoint, stop automation, drag on the map to select position
   and final facing (or enter x/y/heading), then **Go & map**. Wait for verified
   arrival or a reported failure. SLAM stays live; **Save a new map** remains
   explicit. An unexplored point is approached through mapped-free space.
   Failed exploration goals are deferred until other reachable areas have been
   explored, then retried once using the current map. Watch the deferred count
   and final retry pass in the mapping panel. Stop and sensor/cancellation faults
   still halt the run; this policy does not restart failed nodes.
3. Open **Mission** and select that trial's saved map. After placing the target,
   choose an actual model class from the dropdown (no free-text labels),
   physically validated coordinate method and required return
   tolerance. **Load selected map for mission** stops the owned SLAM/Nav2 stage
   before loading AMCL and the mission stack. The console stays open. Do not run
   any second mapping/localization or velocity writer alongside it.
4. Set the assigned initial pose and wait for localization and sensor readiness.
   Start the mission with the UI visible. No manual drive, navigation goals, map
   edits or manual target coordinates are available in stage 2.
5. Completion requires the target report, return within tolerance and the
   stationary hold. `NOT_FOUND`, `FAILED`, or an unverified Nav2 success is not a
   complete mission. Open **Results** and download the trial JSON, preserving independent target and
   return reference measurements outside mission control.
6. Inform the examiner before any restart/intervention. Nothing auto-restarts.
   **New trial** explicitly reloads fresh mapping and resets the current timer;
   previous trial JSON and maps remain in unique directories. A console restart
   in the same host boot preserves timing; a host reboot marks it interrupted.

## Trial evidence and report constraints

Record mapping time, combined completion time, target class/confidence, target
supporting surface, coordinate method, reported and reference `(x,y)`, Euclidean
position error, actual return error, required tolerance, stationary stop, battery,
software commit, map, failures, interventions, and who measured the reference.
Preserve generated logs/images/maps outside Git. Use TESTING's table for results.
A floor can below the 0.1875 m lidar plane requires a validated alternative;
filters cannot establish a missing range or correct systematic calibration error.

The authoritative instructions are the local five-page project PDF. Demonstration
is 70% and the group report 30%. Within the demonstration: completion time 30%,
location accuracy 20%, implementation quality 20%, questions 20%, extras 10%.
The session is 30 minutes including setup, both stages and questions. Submit the
report within 24 hours of the demonstration, maximum 10 pages including cover and
appendices but excluding source code, single column/single spaced, Times New
Roman at least 12 pt, with the repository link. Include architecture, ROS/TF flow,
mapping/localization, exploration/search, target localization, key parameters,
measured results, failures and limitations. TESTING contains software evidence
and partial physical results; no successful full physical mission is yet recorded.
