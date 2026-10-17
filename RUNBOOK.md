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
3. Open the browser. Configure gap blocking while stopped; verify global/local
   radii and the mapping/search clearance. Apply temporary trial limits only in
   idle mapping. Confirm no control fault. Defaults return after restart.
4. Have the examiner confirm intervention rules and return position tolerance.
   Record the deployed local commit and map path in TESTING's trial table.

## Mapping, then mission

1. Begin one external stopwatch at the start of mapping from the assigned pose.
   Keep it running across map saving, launch transition, initial localization,
   and autonomous search. The UI elapsed value is stage-local and cannot replace
   this combined timer.
2. Keep the target absent during mapping. Start automatic mapping through the
   UI; use manual mapping only as the examiner permits. Save into a unique
   bind-mounted directory. Inspect coverage; `MAPPED_PARTIAL` is not a complete
   room map. An automatic timeout save records a partial map after confirmed
   cancellation and does not restart failed nodes.
3. Stop mapping and its launch cleanly. Confirm SLAM has stopped before launching
   AMCL. Preserve the exact saved YAML/PGM pair; never overwrite lab maps.
4. After the target is placed, establish the assigned initial pose. Configure
   gap policy before Start in stage 2. Launch using the saved on-site map, the
   observed target class, and a physically validated localization method.
5. Start the autonomous mission with the UI visible. Do not manually drive,
   choose navigation goals, edit the map, or enter target coordinates. The
   controller records home and chooses its own viewpoints.
6. Stop the combined stopwatch only after a target was reported and the robot
   returned and verified stationary. `NOT_FOUND`, `FAILED`, a partial map, or a
   Nav2 success outside home must be recorded honestly. Inform the examiner
   before any restart/intervention.

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
