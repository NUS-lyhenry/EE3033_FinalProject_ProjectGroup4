# Validation record

This file separates software checks from physical robot evidence. Update it
with measured results after each room or sandbox trial.

## Software validation

- 27 algorithm tests currently pass on the development computer (Python 3.12).
- Eight Humble integration scenarios pass: target found/return; no target;
  failed search goals; sensor loss; stop during goal acceptance; false home
  success; velocity limiting/obstacle stop/deadlines; and automatic-map return
  with a separate map save preserving the original test map.
- The real Nav2 smoke test passes with seven active lifecycle nodes, AMCL
  localization, YOLO image processing, one final velocity writer, and a rejected
  start when motion is disabled. It uses synthetic sensors on a separate domain.
- HTTP console integration passes: HTML, Start/Stop services, session/origin
  checks, autonomous manual-control lockout, and backend motion-disable checks.
- Three browser checks pass: live camera preview; mapping control lockouts;
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
