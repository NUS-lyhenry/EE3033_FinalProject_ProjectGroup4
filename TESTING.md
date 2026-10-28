# Validation record

## Final corner recovery, capture-time TF and simulator checks — 2026-10-28

The original legacy seed-95 scene is saved as
`validation/corner_seed95_legacy_scenario.json` because the upgraded multi-room
generator deliberately produces a different layout for the same seed. The
reported corner reproduced repeated zero-translation commands. Parameter-only,
smoothing and alternate-controller experiments are retained in
`validation/corner_investigation_20261028.json`; they were not adopted.

Automatic mapping now cancels a stalled action before selecting a short
map/costmap-checked recovery goal. Near obstacles it seeks open space; in open
space it can stage a complete safe-grid route toward the stalled goal. A maximum
of six attempts covers mapping and return. Stop, unknown/out-of-map cells, the
configured clearance and fresh-costmap requirements remain in force. The native
legacy-corner run escaped multiple corners and returned within about 0.12 m,
with a verified stationary hold. It honestly saved **MAPPED_PARTIAL** because
coverage remained unresolved; it is not evidence of complete exploration.

Capture-time TF was reproduced failing by fractions of a millisecond before the
matching transform arrived. One bounded nonblocking inference-period retry now
retains that same image and timestamp. Two near-range native runs passed after
the change; the final repeat moved about 0.21 m and reported sigma about 0.073 m.
Bearing-only A/B, occlusion and cropped-target native regressions passed.
The first remote native run exposed an outdated legacy calibration fixture: its
mock Perception omitted pixel-margin and camera-range defaults. The fixture was
updated without weakening production validation; all 6 calibration cases passed.

All **449 Python checks** and **52 browser checks** passed locally, along with
8 mapping-retry ROS tests, 9 common ROS tests and the native recovery cases
(unknown/uncertain rear, transient/persistent blockage, corner, frozen chassis
and cancellation).

A fresh real-YOLO/SLAM/Nav2 workflow produced a MAPPED map and verified home.
Its default 0.22 m / 10 degree camera mission then returned FOUND_UNLOCALIZED;
that failure is retained. Updating the physical camera to 0.31 m / 28.6 degrees
and repeating the mission on that map produced COMPLETE, sigma 0.0998 m against
the requested 0.10 m, and a verified return. This second attempt is explicitly
mission-only and does not replace the failed first attempt.

The live 3x3 random-world smoke test passed generation, random/manual target
placement and physical camera changes checked against actual TF, CameraInfo and
rendered pixels. Geometry tests cover 103 seeds. Autonomous completion in every
complex random world is not claimed. Final metrics and source hashes are in
`validation/final_recovery_20261028.json`. No physical robot was contacted or
deployed during this verification.

## Near-field/occlusion follow-up — 2026-10-28

After PR #7 merged as 7441622, isolated subagent commits 1643036 (camera LOS)
and e6dfd59 (near-field reframing) were integrated on fix/near-field-occlusion.
All 353 Python checks passed, including wall/unknown/corner supercover rejection,
camera mount offsets, range/FOV constraints, source-anchored bearing hypotheses,
near/crop recovery hints and preservation of strict projection validity.
Full ROS/YOLO simulation was not run, consistent with the deploy-first request.
Deployment was attempted only as a read-only connectivity check; robot SSH at
10.221.243.174 timed out, so this follow-up has **not been deployed** or physically
validated. The existing capture release remains the last confirmed deployment.

## Capture diagnostics and full localization budget — 2026-10-27

Release `capture_20261027_62b14551d2` separates translation/arrival-heading
watchdog windows, counts arrival centering toward localization work, and removes
motion-count exhaustion as an early-return condition. No-safe-next-motion now
holds and observes to the work deadline. Per-frame rejection evidence is JSON
safe and bounded, and Mission journals it per run under the assessment session.
All 320 Python checks passed, including the prior 26.9-second/five-goal scenario
continuing observation and terminating at 45 seconds, separate final-heading
timing, diagnostic producers, persisted rejection records and mission isolation.

The user authorized interruption of an active physical trial. It was stopped
through the normal Stop endpoint before replacing the old console. Robot disk
space was exhausted; only regenerable `.o` intermediates were cleared, retaining
source/runtime programs/maps/results. New source was verified and built in its
own release, and the same map/Fast/bottle/hybrid/0.14 m and 28.6-degree camera
settings (current range 0.4–0.8 m) were restored.

A passive real ROS subscription received the new localization diagnostics
(schema 1, original frame timestamp and rejection codes). The UI was verified
Connected/WAITING; initial pose must be re-established before Start. No new
autonomous Start or full simulation was issued. Physical A/B success remains
for the user's upcoming test. See validation/deployment_capture_20261027.json.

## Physical no-baseline follow-up — 2026-10-27

The recovery release physical run did **not** triangulate: four acquire goals
were effectively in-place, localization travel was zero, and only one valid
frame at one site remained. The mission returned FOUND_UNLOCALIZED after 28.3 s
of localization work. This contradicts any inference that the earlier synthetic
A/B success already established physical A/B performance.

Release `parallax_20261027_50f9b06f72` bounds repeated in-place turns, skips
centering no-ops, prioritizes side baselines, and decouples their planning from
unusable floor depth. Actual Nav2 costs filter baseline alternatives before
dispatch. Four independently valid centered/settled frames may be accumulated
within a six-second site window; arriving/settling has an eight-second station
hold. Completed A remains available while travelling to B, subject to existing
observation age and geometry checks. New motion traces record origin, goal,
translation and purpose. All 302 Python regressions passed.

At the user's request this fix was built and deployed before full simulation.
The same reviewed map, Fast profile, bottle/hybrid and 0.14 m precision were
restored. The new UI/API version and healthy native robot stage were checked;
the current new-version trial was already SEARCHING during the check and was
not interrupted. Physical triangulation success is **not yet established**.
Metadata: validation/deployment_parallax_20261027.json.

## Recovery and repeat-search deployment — 2026-10-27

Release `recovery_20261027_6136d65c75` was built on balance6 in a separate
release/overlay, with all 124 uploaded files verified. The API reported this
exact version after switching the stopped console. Current state is WAITING
with no hardware stage loaded; no real mapping/search Start was issued. Existing
Fast profile, 28.6-degree/0.31 m camera geometry and old results were preserved.
Four known legacy maps received corrected copies; the originals remain intact.
The UI was opened Connected with the latest corrected map selected, bottle and
0.12 m precision restored. The corrected map remains unreviewed.

Completed checks: 47 full browser tests and four final focused UI checks; 66
focused retry/workflow Python checks; native full visual-search (including
four-second loss recovery and autonomous bearing-only A/B), bearing filtering,
eight mapping retry cases, nine ROS cases, and three automatic-calibration cases.
The real Nav2/synthetic-chassis recovery test demonstrated one 0.20 m reverse
fallback after repeated corner blockage, normal recovery without reversing when
a transient obstacle cleared, rear-blind map/AMCL-approved reversing, unknown/
uncertain rear rejection, bounded frozen-chassis failure and cancellation.

The native HTTP workflow check initially found a missing stamp_seconds import.
That import and subsequent generation/threshold guards were fixed and covered by
focused checks; its final native rerun was paused at the user's request. The
full new YOLO/SLAM/Nav2 simulation was not started. These are explicit remaining
validation limits, not passes. User workflow preference: deploy and connect UI
after necessary basic checks, then run full simulation while the user can test
the hardware. Simulation remains paused until requested.

Deployment metadata: validation/deployment_recovery_20261027.json. Physical
localization accuracy and recovery dynamics have not been established by these
synthetic tests.

## Joint search, lock and bearing localization — 2026-10-27

Based on main `fbbf0da`, with local implementation changes. The original speed
and clearance remain unchanged. Completed verification:

- 238 Python tests passed on macOS; the final affected native Humble run also
  passed its Python suite. New checks cover reachable wall-back coverage,
  incomplete-map outcomes, geometry-free target lock, no global resumption after
  lock, stale navigation callbacks, pitch correction, correlated factor
  replacement and safe reframing.
- All 43 Chromium workflow/layout tests passed, including the distinct
  FOUND_UNLOCALIZED/SEARCH_INCOMPLETE displays and boolean TF pitch override.
- The full isolated Humble suite passed (camera, visual mission, hardware launch,
  console, ROS callbacks, adaptive search, stationary checks, legacy calibration,
  automatic calibration, mapping retry, workflow, waypoint, native turning,
  recovery, and NavFn gap checks at both map resolutions). Final camera and
  cancellation changes were rechecked through the affected native cases.
- The native Image/CameraInfo/TF/odometry -> bearing -> MapMMSE test deliberately
  clips the bottom, providing no floor coordinates. Two four-frame sites recover
  (1.000, 1.000) m with sigma 0.0596 m, using one joint likelihood rather than
  accumulating correlated pairs. Only the detector is substituted in this test.
- The full real-YOLO/SLAM/Nav2 simulation generated its map and then completed
  target detection, bounded refinement, one report, return and a verified two
  second stationary window. The clean-start mission took 77.4 s, reported
  (0.9811, 0.8816) m with sigma 0.1067 m as best_available, and finished 0.1212 m
  from its recorded home. That run used the floor/MMSE fallback; the bearing-only
  route is verified separately above. Result: validation/simulation_20261027.json.
- Earlier simulation attempts exposed an incorrect start-heading handoff and a
  stale-TF safety stop during return under concurrent test load. They are not
  successful missions. The clean-start rerun retained the existing thresholds.

Deployment at the user's request: the robot built and imported
`joint_20261027_ebbf200d32` from a separate release/overlay. The new console uses
an independent assessment session, preserves the old source/maps/results and
the existing 28.6-degree, 0.31 m, 0.3–1.5 m camera settings. The local physical
robot UI was opened and verified Connected/WAITING with model labels ready.
No mapping or mission Start was issued by the agent. This establishes deployment
and UI connectivity, not physical navigation/localization accuracy. See
validation/deployment_20261027.json.

The deterministic planner-only comparison uses the same candidate pool and
speed assumptions, ideal observations and no inference/localization time:

| Layout | Mean first visibility, greedy -> lookahead (s) | Full route, greedy -> lookahead (s) |
| --- | --- | --- |
| Open | 107.18 -> 106.79 | 506.31 -> 539.79 |
| Divider | 125.19 -> 129.69 | 631.11 -> 633.08 |
| Alcove | 136.10 -> 134.08 | 770.44 -> 643.49 |

All target cells were observed in these cases. Lookahead does not universally
improve speed: retain the measured regressions rather than interpreting the
three-step policy as globally optimal. Reproduce with scripts/benchmark_search.py.

The checked-in scripts/verify_algorithm_preview.py rejects non-simulation
consoles, waits for model labels/fresh status, and uses the real HTTP workflow.
A complete trial needs a fresh map; --reuse-map is only for an unfinished trial
in a world reset to its assigned origin. The simulator uses its own TF-only
10-degree camera; the physical-device preset is the user's provisional 28.6
degrees, not an independent calibration. Physical accuracy remains unmeasured.


## Algorithm preview and soft target precision — 2026-10-27

The local `main` working tree now supports a hardware-free interactive preview
using the real console, managed stage launches, SLAM Toolbox, AMCL, Nav2,
Mapper, Perception, TargetFilter, Mission and VelocityGate. The environment
provides ray-cast lidar, ideal planar odometry and a rendered photographic cup.
The running preview uses actual YOLO26n inference, not injected detections,
target coordinates, navigation action successes or timed mission transitions.
Physical drivers are disabled and ROS DDS is confined to the container on an
isolated domain. Reproduction uses `scripts/Dockerfile.preview` and the commands
in README; the local run reused the existing Humble image with the same added
dependencies. A clean build of the new Dockerfile has not been repeated.

The original `102a43a` run saw the cup but ended `NOT_FOUND`. A regression
reproduced a detection with sigma 0.1369 m, rejected as too uncertain while
`close_view` was still true. The fix uses total uncertainty consistently,
avoids negligible observation moves and pauses failed reacquisition long enough
for another search view to change the observation. Target precision is now a
soft goal: stable settled observations retain their true sigma, and a fresh
confirmed estimate may be reported as `best_available` after bounded refinement.
Invalid geometry, stale/inconsistent observations and Stop cannot use this
fallback. Sensor uncertainty is not divided by the number of correlated frames.

Completed local verification:

- 189 Python tests passed, including simulation ray geometry/occlusion, a closer
  observation for an uncertain target and preservation of correlated uncertainty.
- All 35 browser scenarios passed, including approximate-result labels and the
  original mapping/mission/Stop flows.
- Nine native ROS Mission/filter/gate cases passed together, including a sigma
  0.25 m stable estimate reporting with unchanged variance after its refinement
  budget, then returning and stopping. Stop with a cached soft estimate passed.
  The exact sigma 0.10 m floating-point boundary case passed separately.
- Native Image/CameraInfo/TF perception regression passed: above-goal in-range
  uncertainty remains observable; clipping, invalid frames and out-of-envelope
  contacts cannot manufacture a usable coordinate. Neural inference is replaced
  only in this isolated regression, not in the interactive preview.
- In the full real-YOLO/Nav2 preview, the revised run completed mapping, search,
  reporting and verified return. Report: (0.9678, 0.8964) m, sigma 0.0990 m,
  `preferred_precision`; final home distance 0.1120 m and stationary verification
  passed. Search-through-return took 96.3 s at the original speed profile. The
  combined timer also includes manual map review and initial-pose setup.
- JavaScript/shell syntax, Python compilation and diff whitespace checks passed.

These results are simulated software evidence. They do not validate physical
detection accuracy, balancing dynamics, braking distances or robot deployment.
These checks were run locally; consult GitHub Actions for remote CI status.
No new hardware run or robot deployment is claimed.

## Visual single-target workflow and navigation trial — 2026-10-27

This change retains the automatic frontier mapper and adds shared camera-only
floor geometry, obstacle-aware search costs, bounded target acquisition,
XY-only home arrival and a single-page operator workflow. Legacy calibration is
optional and only explicitly selected profiles affect a mission.

Completed local verification:

- Python 3.12 on macOS: 184 algorithm tests passed. Python compilation and
  diff whitespace checks passed.
- Chromium/Playwright: all 34 browser scenarios passed, including the single
  workflow page, explicit mounting values, fast-profile responses, map dragging,
  phone layout, active-state lockouts and Stop.
- ARM64 Ubuntu 22.04 / ROS 2 Humble / Nav2 1.1.20: overlay build and the full
  isolated suite passed. This includes hardware-launch configuration, HTTP/ROS
  transactions, original mission/search/stationary/mapping/waypoint regressions,
  legacy calibration compatibility, native turning/recovery and NavFn passage
  checks at 0.05 m and 0.01 m resolution.
- Seven visual mission scenarios passed: immediate reliable report, approach
  then report, stopping an approach once a good close view appears, lost target,
  exhausted acquisition budget, Stop during localization, and late approach
  acceptance. They use actual Mission/filter/gate callbacks and synthetic motion.
- The image pipeline passed with actual ROS Image/CameraInfo/odometry/AMCL/TF,
  no lidar subscription and no fitted profile. Neural inference alone is
  substituted, including a 0.30-second delay. Far/clipped targets and mismatched
  camera frames cannot produce report-quality coordinates.
- Native fast turning built commands above 0.45 rad/s under the 0.60 cap. Home
  arrival completed without rotating to a deliberately different final yaw.
  These are synthetic controller results, not physical speed measurements.
- Final HTTP/ROS checks passed for saving measured camera settings, applying a
  mission speed profile without changing its 75-second deadline, setup lockout
  during localization and Stop. Legacy projection checks were rerun successfully.

The reproducible local suite is `tests/run_humble_checks.sh`; its isolated Docker
runtime is `tests/Dockerfile.humble`. Native tests use separate ROS domains and
localhost-only DDS. The local Docker test had networking disabled.

Physical validation has not been performed. Original speed (0.12 m/s,
0.40 rad/s) remains default. The explicit Fast trial profile (0.16 m/s,
0.60 rad/s, shim 0.55 rad/s) and changed rotation thresholds require physical
turn/stopping/localization comparison. No target-location accuracy, physical
coverage, turn-time improvement, complete mission or deployment is claimed.
Independent camera measurements must validate the mount/intrinsics and the
initial 0.35–2.0 m usable range. Report sigma is a model, not an accuracy result.

## Map-point calibration replacement — 2026-10-27

The old image-centred / measured-distance process has been replaced by a fixed
bottle map point and autonomous map-pose collection. Physical deployment and
validation are pending because the robot is unavailable.

Local validation:

- 171 Python unit tests passed, including known-map-point fitting, held-out
  checks, map clearance, both hardware yaw polarities and duplicate proposals.
- 30 Playwright browser tests passed, including current-map bottle selection,
  the dimensions-only Start request, Stop, result display and mobile layout.
- The ROS package built successfully using the local Jazzy overlay.
- Three isolated ROS/HTTP automatic-calibration cases passed on domain 108.
  The real main-console forwarding API, calibration node and final velocity
  gate ran with synthetic sensors and inverted-yaw robot physics. Collection
  saved 17 timestamped map views and recovered a synthetic 14.2° pitch. Invalid
  tokens/origins, stale controls, wrong-map points, edits during a run,
  heartbeat loss and localization jumps were checked. Fresh/stale mapping-pose
  handoff cases also passed.
- Six isolated mission-projection / launch cases passed on domain 106,
  including projection without lidar and rejection of conflicting stacks.
- The clickable simulation completed through an actual browser and HTTP server,
  using the real fitter; its saved profiles remain explicitly simulated.
- Python compilation, JavaScript syntax and diff-whitespace checks passed.

The broader local navigation-console regression cannot run on this desktop:
`nav2_msgs` is missing from its Jazzy installation. The workflow fixture likewise
exits before mapping becomes ready. The repository's Humble CI installs Nav2 and
runs those navigation regressions. These local environment limits do not replace
CI or a physical test of the new calibration controller.


This file separates software checks from physical robot evidence. Update it
with measured results after each room or sandbox trial.

## Robot deployment and reported downward camera tilt

2026-10-25: deployed runtime5c3d5b8 in the robot's existing Humble container and
rebuilt its separate overlay. The live main console is available on forwarded
port8765; its owned calibration stage uses8780. Original maps and trial records
were preserved. The operator subsequently switched to Mapping to construct a
new map before calibration; that work remains under operator control.

The lab image header is `default_cam`, which has no mounting TF. Calibration
now reads the physical translation from `camera_link`, verified on the robot as
(-0.035,0,0.2655)m relative to `base_footprint`. It still retains the actual image
frame in saved profiles. The reported13–15 degree downward tilt initializes the
fit at14 degrees; pitch remains a fitted variable, and the main calibration
result displays its fitted angle. Bottle defaults are0.215m height/0.065m diameter;
the real distance field starts empty for an actual floor measurement.

180 pure tests,11 calibration browser cases and the full virtual ROS/HTTP route
pass, including recovery of a known14.2 degree pitch and the independent image
header/mount frames. Pure fits also recover14.3 degrees from13 and15 degree
starting hints. GitHub [run40190191931](https://github.com/NUS-lyhenry/EE3033_FinalProject_ProjectGroup4/actions/runs/40190191931)
passes all three jobs, including the complete Humble native regression suite.
Live sensors, fresh JPEG preview and bounded balancing capture were checked;
the final command gate was the sole `/cmd_vel` writer. The agent did not start
physical calibration movement. No real bottle range has been entered, so actual
camera pitch and physical localization accuracy have not yet been calibrated.

## Automatic camera setup from one distance

2026-10-23: runtime codeeb42349 adds a mapped-area preparation workflow taking
bottle height, diameter and one floor range. It automatically centres the bottle,
collects 18 timestamped views, jointly fits camera height/focal lengths/pitch/yaw
and bottle bearing, checks two held-out translation positions, and saves a
profile. The main console loads the selected reviewed map and AMCL, reusing only
a fresh robot pose on that exact map when available. Manual measured calibration
remains available in Advanced panels.

178 pure tests and32 browser tests pass. The isolated native ROS test on desktop
Jazzy/domain108 passes automatic HTTP collection through the real calibration
node and velocity gate, a virtual robot, future-dated AMCL TF, profile saving,
heartbeat loss and localization-jump stops. The native test uses320×240 synthetic
images; math and browser checks also cover640×480. Dedicated camera workers keep
encoding/inference and graph discovery outside the ROS control executor.

GitHub Actions [run39951684688](https://github.com/NUS-lyhenry/EE3033_FinalProject_ProjectGroup4/actions/runs/39951684688)
also passes all three jobs: algorithms, browser, and native-navigation on Humble,
including automatic calibration and the existing navigation regression suite.

This version has not been deployed to or physically tested on the robot: the
operator confirmed it is powered off. On-robot Humble verification and a measured
field trial remain pending. Held-out pose checks establish consistency with robot
pose estimates, not independent surveyed map-frame accuracy. No physical motion
was commanded during development.

## Integrated one-time camera setup

2026-10-23: deployed runtime05b6a6b with Camera setup in the main console. The
console owns calibration as a preparation stage, routes Stop to its positioning
service, discovers validated profiles after restart and forwards the latest valid
profile to future missions. Camera-only floor projection is the UI default after
validation unless the operator chooses another method. Calibration does not start
the assessment timer; existing trial records remain preserved.

167 pure checks pass. All29 existing browser cases plus the new preparation-page
case pass; relevant full-workflow/mobile checks were rerun after the default-method
change. All6 calibration integration cases pass on Pi/Humble domain107, including
real main-console Stop, automatic profile discovery and a non-started assessment
timer. Tests also exercise continuously changing JPEGs during blocked inference
and bounded balancing odometry. Live embedded camera, enabled setup controls and
no browser errors were verified with all non-GET browser requests blocked. The
owned calibration stage is idle; no motion was requested by the agent. The camera
profile itself remains to be measured and validated by the operator.

Capture now permits bounded balancing over0.35s (mean v<=.18m/s,w<=.60rad/s,
XYspan<=.15m,yawspan<=.45rad), with fresh sensors/zero final commands and idle
positioning controls. A fresh single-image snapshot replaces the multi-frame box
jitter gate. Mission return verification is unchanged. This does not establish
physical map-frame target accuracy.

## Measured-object camera calibration

2026-10-23: added a separate preparation launch and guided browser UI with live
detector boxes, frozen-image correction, centring and bounded 10 cm movement,
measured bottle/floor samples, robust camera fitting and independent validation.
Saved profiles feed mission floor projection and camera–lidar association.
The clickable simulation uses the same fitter without ROS and marks its profiles
as simulated; mission perception refuses those profiles.

Local checks: 156 pure Python checks pass, including 27 new calibration/demo
checks. The browser suite covers setup, image correction, training, fitting,
validation, saving, Stop, stale sensors/ownership, error display, demo controls
and mobile layout. Four native ROS Jazzy checks pass on isolated domain 106:
real HTTP/sample/fit/holdout/save flow with synthetic camera/YOLO proposals and
the real final velocity gate, browser-loss stop, timestamped map projection,
resolution/frame/range rejection, and corrected camera–lidar association. No
physical-domain motion was commanded. Humble calibration integration is added
to CI. The broader local console integration could not run because this desktop
lacks nav2_msgs; the CI Humble image installs the native Nav2 dependencies.

These results validate software and synthetic geometry, not physical camera
accuracy, lidar extrinsics, or map-frame target accuracy. A real measured-bottle
calibration and independent map reference trials remain required.

## Gap-policy startup recovery

2026-10-23: the live UI retained a driving lock after the global costmap's
parameter service was briefly unavailable during stage startup. Later readback
was complete and synchronized, but the original console did not clear the old
read-only failure. Reapplying the unchanged Bold/gap-OFF policy cleared the
immediate fault while the robot was waiting.

The console now clears a read-only availability fault after complete, validated,
synchronized readback. Inconsistent parameters and unverified update/rollback
still latch a lock until a verified reapply; subsequent read failures do not
overwrite that reason. Stage replacement resets the lock classification.

129 algorithm tests pass. All three native Humble console cases pass on isolated
domain 102. The expanded mapping/mission regression fails on the original code,
then passes with the fix: startup read recovery performs no parameter writes,
and matching reads after an uncertain rollback cannot unlock driving. This is
software evidence, not a physical navigation result.

## Bounded navigation recovery

2026-10-22: the physical mapping logs repeatedly reported no valid path,
followed by the stock Nav2 recovery sequence: fixed 1.57 rad spins, five-second
waits and 0.30 m backups. Some spins/backups aborted on collision checks. These
logs establish the repeated recovery commands, without establishing the physical
cause of the blocked path or proving that every observed turn was a recovery.

The project now installs its own NavigateToPose tree for both stages. It keeps
1 Hz replanning and the existing path controller. A failure refreshes both
costmaps, waits one second for new observations, and retries once. Persistent
failure returns to the mapper/mission instead of initiating recovery spins or
backups. Explicit waypoint-facing Spin and normal collision-checked path turns
remain available. The production controller's 18 s progress allowance remains.

All 129 algorithm checks pass. The real Humble navigator/planner/controller
fixture on isolated domain101 verifies Nav2-only lifecycle reload while its
map/TF/chassis fixture stays running; persistent no-path abort in 1.54 s with
zero movement; successful navigation after a temporary blockage, including
clockwise path alignment from a 2 rad initial heading; frozen-chassis abort
after one retry; and cancellation during the pause with no successor goal.
No recovery Spin or BackUp action and no /cmd_vel publisher occur. Only the
synthetic stall test shortens progress allowance to 1.5 s (4.65 s measured abort).
All eight native mapper retry/cancellation cases and all six native manual
waypoint position/facing cases also pass on isolated domains96 and98.
These are software checks, not a new physical navigation trial.

Source commit `ef78258` was backed up, deployed and rebuilt in the separate
robot overlay. The operator then loaded mapping stage `mapping_riasxm7z`;
readback confirms its navigator selects the installed bounded-recovery tree.
It was WAITING/ready without a console fault, with exactly one final velocity
publisher (`project_velocity_gate`) and 21 sampled zero commands. No navigation
trial was started by the deployment. Saved maps/results remain outside the
source deployment directory. The persistent console was retained, so its
version label still identifies `ddd6d13`; the new navigation launch/tree comes
from `ef78258`. Future mapping and mission stage loads select the new tree.

## Stationary-window verification for the balancing robot

2026-10-22: the Bold run in `mission_2dlna35_` confirmed the bottle at
approximately (0.383,-0.531) m with 90% confidence over four frames, and Nav2
returned to the recorded start. Its frozen software return error was 0.0122 m,
within the selected 0.20 m tolerance. It then failed the old continuous raw-speed
hold. That historical failed result is preserved; no absolute target/return
ground-truth measurement or successful stop verification is retroactively claimed.

The revised stop check uses time-weighted mean absolute linear and yaw speeds
over a complete two-second window, with separate 0.035 m/s and 0.10 rad/s limits.
The odometry XY window span must be <=0.05 m and unwrapped yaw span <=0.12 rad.
Fresh zero final commands are mandatory; nonzero commands, gaps/stale or invalid
odometry reset/prevent verification. The original return tolerance and 12 s
verification deadline remain. Window measurements and specific reasons are
exported in the result and progress is shown in the Results page.

129 algorithm checks and all19 browser checks pass. Ten new native Humble stop
cases on isolated domain100 pass: bounded balancing spikes complete; sustained
translation/yaw, slow creeping/turning, nonzero commands, stale zero commands,
odometry gaps and invalid poses fail; operator Stop remains terminal. The original
nine ROS mission/mapper/gate cases also pass on domain88, including successful
found/return, absent target and rejection of a false Nav2 success outside home.

A read-only shadow check on the real stopped robot accepted a fresh two-second
window (101 samples): mean linear 0.0346 m/s, mean yaw 0.0267 rad/s, XY span
0.0499 m, yaw span 0.0266 rad. Current map-frame home distance was 0.0718 m;
no nonzero movement commands were observed. This validates acceptance of current
balancing data, without starting a mission or changing its historical result.
A complete physical mission using the new verifier is still to be demonstrated.

Runtime commit `ddd6d13` was deployed and the existing separate overlay rebuilt.
The persistent console and failed mission were shut down cleanly, then the same
reviewed map, bottle/lidar selection and assessment record were restored. Live
mission status exposes separate linear/angular readings and the new window
field, reports WAITING/ready, and retains Bold. SHA-256 checks confirm the prior
failed mission result, map YAML/image and installed lab base configuration are
unchanged. The prior HTTP result is also archived in the assessment directory as
`pre_stop_window_fix_ddd6d13.json`. No autonomous mission was started during deployment.

## Bold as the mapping and search default

2026-10-22: at the operator's request, every new mapping and mission stage now
defaults to Bold with gap blocking OFF. Mapper/search clearance and turn stop
are 0.14 m; the controller uses heading progress of 0.20 rad, open-loop command
ramping and 0.30 m path sampling. Cautious and Balanced remain selectable.
Speed caps, collision radius/padding, scan mask and forward stop are unchanged.

All 116 algorithm checks pass, including a consistency check against the actual
project/Nav2 YAML. All 18 browser scenarios pass, with the two affected cases
rerun after correcting their expected default and waiting for stage loading.
The browser flow verifies that mission loading and mapping reloading restore
Bold after selecting Cautious. Three native Humble console cases also pass on
isolated domain94, checking synchronized Bold startup in both mapping and mission,
all profile transactions, rejected writes/rollback and active-state lockout.

Deployed runtime commit `cd6fb62` into the existing robot overlay. The failed
search and persistent console were stopped cleanly and reloaded without starting
navigation. The reviewed map `onsite_6dmqc_95/map.yaml`, bottle/lidar selection
and current assessment record were retained. Live `/api/gap` confirms synchronized
Bold, reserve OFF, search/turn clearance 0.14 m, heading progress 0.20 rad,
open-loop rotation and path sampling 0.30 m. The collision radii remain 0.14 m
with 0.01 m padding. AMCL and all required sensors are healthy; mission is
WAITING/ready with no fault. The served UI identifies Bold as the default and
reports software version `cd6fb62`. No physical Bold search result is claimed.

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
GitHub run39739555918 passed all three jobs, including the full native suite.

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
