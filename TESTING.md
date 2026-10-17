# Validation record

This file separates software checks from physical robot evidence. Update it
with measured results after each room or sandbox trial.

## Software validation

- 53 algorithm tests currently pass on the development computer (Python 3.12),
  including scan self-occlusion handling and bounded navigation trial settings.
- Nine Humble integration scenarios pass: target found/return; no target;
  failed search goals; sensor loss; stop during goal acceptance; false home
  success; velocity limiting/obstacle stop/deadlines; and automatic-map return
  with a separate map save preserving the original test map; and confirmed timeout cancellation before a partial-map save.
- The real Nav2 smoke test passes with seven active lifecycle nodes, AMCL
  localization, YOLO image processing, one final velocity writer, and a rejected
  start when motion is disabled. It uses synthetic sensors on a separate domain.
  Native Humble Nav2 trial parameter updates and readback also pass (domain 95).
- Three HTTP/ROS console integration cases pass on Humble (domain 94): HTML,
  Start/Stop services, session/origin checks, autonomous lockout, motion-disable,
  live trial readback/limits/rollback/active lockout, battery freshness, and narrow-gap
  transactions in both stages including rejected writes and unverified rollback lockout.
- Six browser checks pass: live camera preview and motion-state display;
  mapping control lockouts; slider application and stale battery display;
  target/confidence/map/return-result rendering; and gap toggle readback/active
  lockout in both stages. Mock results are explicitly
  labeled simulated and are not physical trial evidence.

Native Humble narrow-passage tests pass on isolated domains 91 and 92. Both
costmaps use synthetic static obstacles and the actual project inflation policy;
NavFn computes paths through the synthetic room. This does not validate physical
body geometry or sensor visibility.

| Grid | Opening | Reserve OFF | Reserve ON |
|---|---|---|---|
| 0.01 m | 0.34 m | Path succeeds; center cost 224 | No path; center cost 253 |
| 0.05 m | 0.30 m | Path succeeds; center cost 248 | No path; center cost 253 |
| Both | 0.70 m | Not separately tested | Path succeeds; center cost 0 |
| Both | 0.20 m | No path; center cost 253 | Not separately tested |

Each run switches OFF again and verifies the narrow passage reopens. Costs are
checked in both global and local costmaps before invoking the actual NavFn action.
No chassis output publisher exists in this fixture.

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

Continuation on 2026-10-17: deployed the narrow-gap console and verified its
real Apply flow (both costmap radii 0.18 m; mapper clearance 0.26 m). A short live
mapping attempt failed before sending a navigation goal because no connected
space satisfied this clearance at the current starting pose. It was explicitly
stopped. A 30 s monitor recorded 607 `/cmd_vel` messages, all components zero;
no Nav2 commands were observed. This is a rejected physical trial, not room
mapping or successful return. Its separate snapshot is
`/home/ros2_ws/group4_maps/onsite_sd24jtnz/map.yaml`; this is a failed-start
snapshot, not a complete room map. Screenshots `live-gap-enabled.png`,
`live-gap-mapping-trial.png`, and `live-gap-stopped.png` are local ignored artifacts.

At rest, a six-second read-only diagnostic found median timestamp ages of
approximately 1 ms raw lidar, 5 ms adapted lidar, and 3 ms odometry (maximum
27 ms). The rear raw scan had a 0.052 m minimum; after the existing self-mask,
one rear point remained near 0.14 m. During the 30 s monitor, 246 of 299 adapted
scans had a minimum below the unchanged 0.16 m rotation threshold. The physical
source of that return remains unverified. The displayed detector labelled the
Pepsi can `cup` at approximately 0.57 confidence in one sampled image; this is
below the project filter's 0.60 threshold and is not confirmed target localization.
The can's supporting surface/height remains to be confirmed.

The robot was left stopped with the console open. No collision threshold, scan
mask, balance PID, or original lab program was changed. Source backup on the Pi:
`/home/balance/ros2_ws/group4_backups/pre_gap_source_20261017.tar.gz`.
Publishing remains blocked: Git CLI lacks HTTPS credentials, and an actual
Contents write through the connected integration returned HTTP 403 despite
repository metadata reporting `push: true`. No source upload succeeded.

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
