# Validation record

This file separates software checks from physical robot evidence. Update it
with measured results after each room or sandbox trial.

## Hardware launch configuration regression

2026-10-22: a physical mission received the assigned initial pose but could not
localize. Its scans used `laser`, while the base mounting transform used the
fallback `laser_link`. The mission's generic `config` argument was inherited by
the included lab base launch, so it read the project YAML instead of its own
installed base settings. Mapping had no parent `config`, which explains why
the earlier mapping stage worked.

The hardware wrapper now explicitly passes the installed base configuration and
scopes each driver include. No lab files, PID settings or saved maps are edited.
`tests/integration_hardware_launch.py` reproduces the conflicting parent argument
with the real launch engine and a driver stand-in that reads YAML without opening
hardware. It checks both use of the lab frame/settings and preservation of the
parent project configuration. The unmodified code fails the first check; the fix
passes both checks on local Jazzy and the robot's Humble. All 115 algorithm tests
also pass. The native CI job now includes these two launch checks.

The fix was deployed to the existing robot overlay and only the idle mission
stage was reloaded, retaining the operator's reviewed map
`onsite_6dmqc_95/map.yaml` and bottle/lidar selection. Reapplying the previously
selected start produced fresh AMCL output, a valid map-to-base transform and
`ready: true` with no console fault. Sample covariance was about 0.03 m² in x/y
and 0.12 rad² in heading. A read-only sample observed one final velocity
publisher (the project gate) and 11 zero-command messages. The search was left
waiting for the operator. This establishes working live localization; it does
not measure absolute pose error or demonstrate bottle finding and return.

## Follow-up turning policy validation

2026-10-22: Balanced and Bold now include graded turning policies as well as
planning clearance. Local validation passes 97 pure Python tests and all 13
browser checks. The turn gate enforces a 0.14–0.16 m configuration range and
rejects changes while enabled. API readback verifies the actual progress-checker
plugin, controller settings and gate as part of the same rollback transaction.

`tests/integration_turning_nav2.py` exercises the real installed Nav2 shim and
pose progress checker with a synthetic chassis that under-reports angular
velocity: original Cautious progress failure; Balanced and Bold turn-then-drive;
abort when physically frozen; and Bold stopping for a closer obstacle. Synthetic
tests run on separate domains without a `/cmd_vel` publisher. Production keeps
the 18 s progress allowance; the regression fixture shortens it to 2 s.

The robot was disconnected for battery charging before deployment. GitHub run
39720335277 passed all three jobs for commit44c032f. Its isolated Humble container
verified native console transactions, all five real-Nav2 turning cases, and
passage checks at both resolutions. Balanced/Bold completed turn-then-drive;
Cautious exposed the original stall, and frozen/close-obstacle cases aborted.
This is synthetic software evidence. The follow-up turning policy has not yet
been deployed or physically demonstrated on the car.

## Operator pages, model labels and mapping waypoints

Local algorithm checks pass 115 tests and all 17 browser checks pass, including
subpage navigation, model selection and waypoint drag/facing. The UI uses a
target dropdown populated from the configured weights. Metadata probe, missing
weights, class validation, cache invalidation and probe shutdown have isolated
tests; no class list from a different model is substituted when weights are
unavailable. Model metadata on the charging robot has not yet been read with
this new endpoint.

Stage-1 waypoints retain both position and final facing. Pure checks cover
unknown-space approaches, exact reachable targets, occupied/outside/nonfinite
rejection, rotated map coordinates and deferred targets. Native waypoint tests
cover HTTP-to-mapper navigation and Spin, independent pose/heading verification,
false action success, Stop during late Spin acceptance, stale requests after
Stop and stage-2 rejection. The expanded CI also reruns the original mapper/
mission, retry and workflow regressions. GitHub run39721950629 passed all three
jobs for feature commit e70437c: 115 algorithm checks, 17 browser checks, native
console3, original ROS9, mapping-retry8, workflow2, waypoint6, real-Nav2 turning5
and both passage-resolution fixtures. The waypoint cases verify requested
position/facing and repeated goals, invalid goals/stage2 rejection, false heading
and position success, Stop during late Spin acceptance and stale requests after
Stop. These are synthetic checks; no physical waypoint or complete mission result
is claimed. Follow-up mobile/header and mapping-layout checks also passed locally.

## Software validation

2026-10-22: the basic-requirements workflow reuses the existing mapper, mission,
detector, target filter, velocity gate, Nav2 configuration and installed lab
drivers. A persistent console now coordinates the two stages.

- 94 pure Python tests pass on the development computer (Python 3.12), including
  scan handling, navigation bounds, continuous timing, atomic results, saved-map
  validation/review, owned process shutdown and cancellation without auto-restart.
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
- Thirteen browser checks pass: complete two-stage map/save/review/pose/search/return
  flow, simulated read-only preview, active-stage restrictions, false-success
  rejection, stale/offline telemetry, retained optional controls, Stop during
  stage loading, deferred-frontier/retry display, and mobile layout. Fixtures are simulated; they are not physical
  trial evidence.
- Two native Humble workflow checks pass on isolated domain 93: HTTP controls
  launch real mapper/mission/filter/gate processes with synthetic sensors and
  navigation actions, save through the real Nav2 map saver, reject unreviewed or
  previous-trial maps, enforce initial pose and stage restrictions, persist the
  verified return and combined timing, and prevent a late launch after Stop.
- Native Nav2 smoke was rerun on domain 95. The original test missed a response
  during DDS discovery although all seven nodes activated; bounded retries of
  the read-only lifecycle query resolved the test stall. The smoke then passed.
- Eight additional native Humble mapper checks pass on isolated domain 96:
  failed area A is deferred while B is explored, then A is retried; timeouts and
  late goal acceptance wait for terminal cancellation before any successor;
  repeated failure receives only one retry pass and saves a partial map; Stop
  during cancellation prevents resumption; unconfirmed cancellation and sensor
  loss halt exploration; and the mapping time limit prevents further exploration
  or retries. These use synthetic frontier/action fixtures and establish policy,
  not successful physical exploration. The original nine ROS scenarios and two
  workflow checks are also rerun for this change.

2026-10-22 audacity controls: Cautious retains the original 0.22 m planning
clearance; Balanced selects 0.18 m and Bold 0.14 m. The additional pure checks
cover both stages, both reserve states, invalid profiles, inconsistent readback,
unchanged collision envelopes and connectivity through 0.30/0.40/0.50 m synthetic
corridors. Browser checks exercise every tier in both stages, reserve retention,
active lockout, rejected changes and mobile layout. The three native Humble
console cases were rerun on isolated domain 94, including all six policy
combinations in mapping and mission, rollback after a simulated write rejection
at each level, legacy toggle retention and active-state rejection. These tests
do not issue motion commands on physical domain 6. Physical passage performance
at Balanced/Bold has not been demonstrated.

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

The audacity change was also checked with native NavFn at both resolutions:
the original narrow passage remains passable with reserve OFF at all three
levels, becomes blocked with reserve ON even at Bold, and a 0.20 m passage
remains blocked at Bold. Both costmaps retain their original collision radius
and padding. This verifies that reducing the planning reserve does not reduce
the Nav2 collision envelope.

## Physical validation

2026-10-22 audacity deployment: the operator explicitly requested saving the
current map and restarting. Mapping was stopped and the map saved separately as
`onsite_6h8dszmj/map.yaml`; it remains listed in the console catalogue. The
previous source was backed up as `pre_audacity_d35497a_20261022.tar.gz`, then
commit `d35497a` was deployed to the existing project overlay. The console and
owned mapping stage were restarted, a fresh trial was loaded, and live readback
verified Cautious at 0.22 m with gap blocking OFF, both costmap radii 0.14 m,
healthy map/lidar/odometry and no control fault. The stage waits for operator
Start. No autonomous run at Balanced/Bold was performed. GitHub CI passed both
jobs for the feature commit.

Immediately before deployment, a read-only 6 s lidar/command diagnostic sampled
60 scans; 11 contained a usable return below the unchanged 0.16 m rotation
threshold. The nearest return was approximately 0.14 m at 214 degrees. Nav2 was
requesting turns and the final gate passed rotation between close returns. The
physical source of this rear return remains unverified; audacity levels do not
change the turn check or lidar mask.

2026-10-22: robot rediscovered at its current LAN address, existing Humble
container restarted, and the assessment UI deployed to the separate project
folder. Original lab programs, PID and maps were preserved. Source backup:
`/home/balance/ros2_ws/group4_backups/pre_basic_a279a8e_20261022.tar.gz`.
The live UI was opened in Firefox. With the target removed and operator readiness
confirmed, one frontier goal succeeded, but the mapping return timed out. The
93.8 s run saved `onsite_6cw1axr_/map.yaml` as **partial** in the persistent map
directory. The operator confirmed unexplored areas remained. After explicit
Stop, 499 sampled final velocity messages were zero; the gate was the sole
publisher. Battery was approximately 11.24–11.27 V.

A 25 s moving diagnostic saw rear as well as front returns across a 359.2 degree
scan. Rotation was blocked by the existing 0.16 m clearance check in 36/244 scans;
the physical source of close rear returns remains unresolved. Separately,
replaying the saved map found diagonal frontier chains split into fragments of
at most four cells and discarded by the minimum-five-cell filter. Eight-neighbour
frontier grouping fixes that loss while retaining four-neighbour route
reachability and the same footprint clearance. A synthetic slanted-boundary
regression and all nine native ROS scenarios pass.

The second physical mapping trial used commit `d9104c2`. It reached its first
goal and selected a second frontier, demonstrating that the diagonal-boundary
fix continued exploration. Known map cells increased from 2004 to 2242 during
the recorded interval. The second goal timed out after intermittent turn
blocking; a 25 s diagnostic found close returns below the unchanged turn
threshold in 184/240 adapted scans. The 75.7 s trial saved another partial map,
`onsite_kk4zlmos/map.yaml`, and was explicitly stopped. Its latest grid is
83 by 63 cells at 0.05 m/cell. Four unknown-boundary groups span approximately
0.30 m or more, with two smaller groups; boundary extent does not establish
passage width or traversability.

A stationary 10 s raw/adapted scan comparison found persistent close returns in
the rear sector, approximately raw angles 150–215 degrees. Their physical source
has not been established. The operator did not identify an obvious nearby
object; actual chassis dimensions and lidar clearance are still needed to
validate any additional self-return mask. No collision threshold was lowered,
no extra points were masked, and no further motion was started. Complete mapping,
physical target localization and the full target-search/return remain pending.

GitHub publication succeeded on 2026-10-22 after device authorization added the
workflow scope required for the repository's CI file.

The deferred-goal policy was deployed as `5f14407`. The first stack startup left
SLAM alive but unresponsive to repeated parameter requests, with no initial map;
the controller answered normally and a Fast DDS shared-memory lock error was
observed. No navigation goal was sent. After explicit shutdown, the installed
`fastdds shm clean` utility removed two zombie ports and two zombie segments,
preserving one live port/segment. A manually reloaded stage then produced a map.

In the subsequent live run, the mapper canceled a timed-out second frontier,
deferred it, checked the other available approaches, and entered its single
retry pass when none qualified. Nav2 reported that the replanned retry succeeded;
three exploration goals were issued and two reported reached. The map's known
cells increased from 1180 to 3426. Operator Stop interrupted the return, followed
by a brief Start/Stop. The operator then reported that the robot had been stuck
against a mattress and the map had become wrong. **This run is invalid as mapping
or localization accuracy evidence.** It demonstrates the retry/cancellation
sequence only; no correct complete map, physical goal accuracy or return success
is claimed. The operator requested a fresh map using the UI's **New trial**,
which restarts owned mapping/SLAM and resets the current trial timer while
preserving previously saved maps and archived records.

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
At that time publishing was blocked: Git CLI lacked HTTPS credentials, and an actual
Contents write through the connected integration returned HTTP 403 despite
repository metadata reporting `push: true`. On 2026-10-22, GitHub CLI device login
completed as the repository owner and the Git push dry run succeeded. Final
publication and fresh physical results are recorded separately below.

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
