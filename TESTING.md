# Validation record

This file separates software checks from physical robot evidence. Update it
with measured results after each room or sandbox trial.

## Software validation

- 40 algorithm tests currently pass on the development computer (Python 3.12),
  including scan self-occlusion handling and bounded navigation trial settings.
- Eight Humble integration scenarios pass: target found/return; no target;
  failed search goals; sensor loss; stop during goal acceptance; false home
  success; velocity limiting/obstacle stop/deadlines; and automatic-map return
  with a separate map save preserving the original test map.
- The real Nav2 smoke test passes with seven active lifecycle nodes, AMCL
  localization, YOLO image processing, one final velocity writer, and a rejected
  start when motion is disabled. It uses synthetic sensors on a separate domain.
  Native Humble Nav2 trial parameter updates and readback also pass (domain 95).
- Two HTTP/ROS console integration cases pass on Humble (domain 94): HTML,
  Start/Stop services, session/origin checks, autonomous lockout, motion-disable,
  live trial readback/limits/rollback/active lockout, and battery freshness.
- Four browser checks pass: live camera preview and motion-state display;
  mapping control lockouts; slider application and stale battery display;
  and target/confidence/map/return-result rendering. Mock results are explicitly
  labeled simulated and are not physical trial evidence.

## Physical validation

2026-10-17: deployed the package in its separate Humble overlay. The live browser
UI displayed the current camera and new SLAM map. Battery was approximately
11.88 V; the base, lidar, SLAM, and Nav2 started successfully. The final velocity
gate was verified as the sole `/cmd_vel` publisher.

The first autonomous mapping start was rejected by the footprint-clearance
check before any navigation goal was sent. Live lidar showed returns down to
0.044 m behind the scanner and approximately 0.15–0.21 m in forward/right
sectors. The robot remained stopped; repositioning or inspection for self
returns was requested. This is a blocked physical trial, not successful mapping.

Subsequent trial: masked returns inside the existing 0.14 m robot radius as NaN
(occluded, not free space), added heading alignment and frontier scoring, and
retried with obstacle protection intact. One retry reached a goal timeout with
intermittent 0.14–0.16 m rear returns blocking rotation. A later retry issued
nonzero gated commands, moved approximately half a metre according to SLAM,
and increased known cells from 1312 to 1609 in the sampled interval. It was
stopped during return to add the requested trial UI. A separate partial map was
saved as `/home/ros2_ws/group4_maps/onsite_wu1owxh0/map.yaml` on the Pi's bind
mount; original lab maps were not replaced. This is not a complete room map.

The updated live UI was opened on the desktop and visually inspected. It showed
real battery readings around 11.70 V, live navigation limits of 0.12 m/s and
0.40 rad/s, and a 60 s per-goal timeout. The underlying software checks do not
establish safe maximum physical speeds or battery capacity.

The real browser Apply flow subsequently set and verified 0.16 m/s, 0.50 rad/s,
and 60 s. A physical retry completed two frontier goals, but the return attempt
hit its navigation timeout at approximately 155 s total elapsed. The robot was
stopped and another partial snapshot was saved separately at
`/home/ros2_ws/group4_maps/onsite_zh1zopew/map.yaml`. Original lab maps and balance
PID were unchanged. The eight isolated ROS integration cases were rerun on
domain 96 and passed. Complete room coverage and physical return remain unverified.

Pending: fresh autonomous room mapping, Pepsi-can detection/class check, map-frame
coordinate checks against reference positions, full target-search/return run,
return stopping distance, and operator-console use during the run.

No accuracy or full physical-mission success is claimed by the software tests.

## Trial record template

| Field | Record |
|---|---|
| Date / software commit | |
| Environment and map path | |
| Target / actual detector class | |
| Mapping time / final status | |
| Reported target `(x,y)` | |
| Independently measured reference `(x,y)` | |
| Euclidean target error (m) | |
| Return distance / required tolerance (m) | |
| Search time / total completion time | |
| Failures / interventions | |
