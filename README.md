# EE3033 Final Project — Group 4

ROS 2 Humble software for a balancing car to map the on-site sandbox, find one
floor target, report its map-frame `(x, y)` position, and return to the assigned
start. The target is absent during mapping. No assessment map or target position
is encoded in the software.

## Operator workflow

Launch inside the existing `ros2_humble` container after building the overlay:

```bash
cd /home/ros2_ws/group4_project
bash scripts/build_robot.sh
source /home/ros2_ws/group4_ws/install/setup.bash
export ROS_DOMAIN_ID=6
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
bash scripts/check_environment.sh
ros2 launch ee3033_project assessment.launch.py allow_motion:=true \
  map_directory:=/home/ros2_ws/group4_maps \
  session_directory:=/home/ros2_ws/group4_results/assessment
```

Open `http://127.0.0.1:8765` through the existing SSH forwarding. The default
single-page workflow is:

1. **Start automatic mapping**. If necessary this loads the mapping stack, waits
   for readiness, then starts exploration. Loading the mapping stack starts the
   continuous assessment timer.
2. Mapping returns and saves a new map. Inspect its accessible and missing areas.
   Select the actual target class and examiner's return tolerance, then press
   **Confirm map & prepare mission**. Partial maps stay labelled partial.
3. Place the target as instructed and establish the assigned initial pose by
   dragging on the map or entering the robot's start pose. Wait for localization.
4. **Start autonomous mission**. The robot searches, confirms the target, reports
   once, returns, and verifies a stationary stop.
5. Read the result and **Download trial JSON**. **New trial** preserves earlier
   maps/results and starts a fresh mapping trial.

Stop is always available. It invalidates queued controls and prevents a late
stage load or navigation acceptance from reopening motion. Crashed nodes are not
automatically restarted. Follow the examiner's intervention procedure.

**Advanced** contains manual/waypoint mapping, map selection, speed profiles,
clearance, camera mounting and diagnostics. Manual drive/goals are stage-1 only;
initial pose and setup are locked during the autonomous mission. Legacy page
links remain available, but normal operation needs no page switching.

## Target lock, hybrid localization and search completeness

The default `localization_method:=hybrid` keeps the shared map-constrained MMSE
estimator and adds timestamped bearing intersection. Two consistent distinct
visual detections lock the one target even when depth is unavailable. After
cancelling the search goal, the mission only reframes/centres this target and
collects local observations; losing depth or the target does not restart map
traversal. A fresh stable estimate meeting the selected soft precision goal
returns immediately. Refinement is bounded to 45 seconds of observation, centering and local
reacquisition, three observation positions. Motion counts are recorded but cannot end refinement
before its localization-work budget is used. Navigation
to observation positions is timed separately; goal deadlines, the overall
mission deadline and the return-time reserve still apply. A valid lower
precision estimate remains labelled `best_available`; no usable coordinates
produces `FOUND_UNLOCALIZED` after a verified return.

At least four settled, horizontally centred frames are collected per bearing
site. A/B/C geometry uses continuous odometry, camera offsets and residual
pixel angles, rejects intersection angles below 15 degrees and map-to-odometry
jumps, and replaces the correlated floor measurements in MMSE rather than
counting the same images twice. Vertical clipping can preserve a bearing;
horizontal clipping only supplies a bounded retreat hint. Detector class alone
still cannot establish brand identity or distinguish arbitrary identical objects.

`config/vision.yaml` now selects the user's provisional 28.6-degree downward
pitch with `use_pitch_override: true` and TF translation. This is not a surveyed
calibration. Set the flag false to use the complete installed TF. A positive
manual height retains the existing complete manual mount. Saved session settings
or explicit legacy profiles still take precedence; inspect the effective pitch
and source in the console. The simulated world has its own explicit TF-only
configuration and retains its independently specified 10-degree camera.

Before target lock, a bounded three-step beam search ranks early new coverage,
travel, turning and observations. It keeps diverse candidates, repairs coarse
coverage holes at 0.325/0.15 m spacing and asks Nav2 to verify disconnected
components. Green/gold/red map overlays show observed/pending/blocked samples.
Candidate exhaustion and failed navigation are not empty-space evidence.
`SEARCH_INCOMPLETE` distinguishes a partial map, blocked/missing regions or an
exhausted budget. `NOT_FOUND` requires at least 98% of the full target-cell set,
no unresolved blockage and no missing connected patch of 0.09 square metres.
These labels describe search evidence, not proof of object absence.

Planner-only comparisons can be reproduced with
`PYTHONPATH=src/ee3033_project python3 scripts/benchmark_search.py`.
Results depend on layout: lookahead is not guaranteed faster than greedy.
See `validation/search_benchmark_20261027.json` and `TESTING.md`.
For a real-YOLO simulation workflow check, run
`python3 scripts/verify_algorithm_preview.py --url http://127.0.0.1:8792`;
this script refuses a physical-robot console.

## Camera-only target localization

The explicit `localization_method:=ground_plane` uses the visible bottom contact
of a target resting directly on a flat floor. Target depth does not use lidar.
Lidar still supplies SLAM, AMCL, navigation and obstacle protection.

No bottle fitting or multi-position collection is required. The camera model
uses matching `CameraInfo` and the image-timestamp transform from
`project_camera_optical` to `base_footprint`. Full TF rotation includes pitch
when the installed driver supplies it. Keep the real camera mount/optical axes
correct; a mathematically valid default CameraInfo is not proof of accuracy.

The map and final target report share one **map-constrained MMSE** estimator,
`/project/target_estimate`. It uses valid, settled camera observations over an
8 s / 30 sample window and computes the posterior mean and 2D covariance on the
loaded occupancy grid. Occupied cells (>=65) have zero prior weight; unknown
cells retain weight 0.25. The uninflated component containing the localized
robot defines possible target space, so near-wall targets are not excluded by
the robot's navigation radius. This prior is conditional on the **loaded map
extent**, which must cover the evaluation arena; it does not establish that an
incomplete map covers the full physical arena.

For observations z_i with covariance C_i, the likelihood combines inverse
covariance weighting with a conservative common-error model: 20% of C_i is
independent, and an isotropic common variance envelope retains 80% of the
largest input eigenvalue. Observed residual scatter is also retained. The
80% split is a configurable model assumption, not a calibrated camera-error
measurement. Thus repeated video frames cannot manufacture arbitrarily high
precision. Wrong camera geometry or pose bias still requires correction.
The grid integrates prior times likelihood using log-space normalization and
adds cell quantization variance. If a nonconvex region places the mean in an
obstacle, the reported point is the nearest supported cell (the constrained
minimum squared-error decision); its displacement from the raw posterior mean
is included in covariance. Both values and the projection flag remain in the
report. A nearest-support Mahalanobis distance above 5 sigma flags severe
map/observation conflict: it is displayed but cannot be accepted. A very broad
likelihood remains eligible; low integrated overlap mass alone is not a conflict
and does not impose a hidden sigma ceiling.

The dashed circle shows the largest principal one-sigma uncertainty, **not** a
guaranteed radial error bound. A position appears after one valid observation;
acceptance requires four fresh consistent observations. At acceptance, the
same estimate ID, coordinates and covariance are frozen for display and
reporting. Unaccepted lost estimates fade and expire after 30 s. **Measure
distance** supports two clicks or a drag without sending pose or motion commands.

If the installed mounting TF is unsuitable, enter measured height, downward
pitch, yaw and horizontal offsets under Advanced. Height zero selects existing
TF; a positive height selects the fixed measured mount. UI angles are degrees,
configuration angles are radians. Zero horizontal FOV selects CameraInfo; a
nonzero, explicitly measured FOV is an approximate override when intrinsics are
unavailable or known to be unreliable. No focal length or camera height is silently invented.

Settings are saved as `vision.yaml` under the session directory and take effect
on the next mission load. For a standalone launch, edit a copy of
`config/vision.yaml` and pass `vision_config:=/absolute/path/vision.yaml`.

The initial reliable range is 0.35–2.0 m. Both localization and search use image
borders, horizontal/vertical field of view, mount offsets, downward pitch and
the near blind zone. Pixel, height, pitch and AMCL uncertainty contribute to the
reported sigma. A 0.10 m sigma is the preferred precision. Stable estimates
above it remain available for bounded refinement and an explicitly labelled
best-available report; their uncertainty is preserved.
Four associated distinct frames are required; repeated measurements retain the
sensor uncertainty floor. Ground-plane reports also require fresh, settled
odometry, limiting errors from balance-car motion. Uncertain detections remain
available to the mission for another observation.

This sigma is an engineering model, not a measured accuracy guarantee.
Independently check target positions at several distances and bearings. Flat
floor/contact assumptions, occlusion, distortion, lighting, model labels and
unmeasured body pitch remain limitations. A fixed mount override cannot recover
unreported dynamic pitch; use stationary observations and conservative ranges.

Observation motion is checked over a 0.6 s odometry window ending at image
capture time. Bounded balancing oscillation can pass even when individual
velocity samples exceed the mean limit. Translation/heading drift and stale
odometry still reject coordinates, and motion span contributes to sigma. A
visible target whose valid projection is waiting for that window is deferred;
it does not count as a new confirmation or as a detector miss. A real miss still
breaks confirmation. Localization waits for at least 2 s, and an unresolved
centered target can use the remaining bounded acquisition time. Unresolved empty visual viewpoints get one deferred retry after other viewpoints;
a locked target instead finishes local refinement and returns.

The installed `usb_cam` 0.8.1 has an epoch conversion defect in
`get_epoch_time_shift_us`: `timeval.tv_usec` is already in microseconds. Dividing
it by 1000 introduces a startup-dependent timestamp error of up to one second.
For that exact version, obtain the official `ros-drivers/usb_cam` tag `0.8.1`
(commit `cb2ae6bc0a312de629f84fef933fb3e07dac1ef8`) and run:

```sh
bash scripts/build_camera_clock_fix.sh /path/to/usb_cam-0.8.1 \
  /home/ros2_ws/group4_driver_fixes/usb_cam-0.8.1/fixed
```

This builds a separate matching camera library and checks the converted clock
against real time. Hardware launch uses it only for the camera when that file
exists; `GROUP4_USB_CAM_LIBRARY_DIR` selects a different explicit location.
Restart the stopped mission stage to reload the camera. Apt-owned drivers and
base/lidar configuration are preserved. Verify raw frame age before motion;
do not conceal an unverified camera time offset by changing image stamps in the
perception node. CameraInfo, physical mounting and floor contacts still need
independent agreement; a clock repair cannot validate a mounting angle.

Existing YOLO weights are loaded from `/home/ros2_ws/weights/yolo26n.pt`; weights
are not committed. The class dropdown comes from that model's actual labels.
The default is cup. A class detector does not establish a Pepsi brand/instance.

[Legacy calibration](CALIBRATION.md) and `localization_method:=lidar` remain
explicit alternatives. Saved calibration profiles are not automatically adopted
by mapping/mission; pass `calibration_file:=...` to opt into a legacy profile.

## Navigation and mapping

The default follower subclasses Humble Regulated Pure Pursuit (RPP), without a
rotation shim. A thin adapter preserves the 0.8 rad/s² command-space turn ramp
when measured angular velocity remains below a motor deadband. At the final
heading, actual pose changes and queued-command delay determine when to brake;
bounded corrections require an actual stationary hold. Actual TF/pose and all
native collision checks remain authoritative. A single navigation supervisor owns cancellation, progress checks and at
most two recovery rounds. `/navigate_to_pose` remains the public action; Nav2's
five internal action endpoints are isolated under `/project/nav2_navigate_to_pose`.
The planner proxy similarly retains `/compute_path_to_pose` and publishes only
full-footprint-checked routes on `/plan`; unvalidated Navfn output is recorded
separately on `/project/native_plan`. Unsafe routes may be replaced by a complete
known-safe grid route, never by an unchecked straight connector.

The final velocity gate preserves curvature when applying speed limits. The
wheel-inclusive measured balancing chassis is 0.115 m long (forward) and
0.18 m wide (across the wheels), conservatively enclosed by
a 0.106801 m body circle plus 0.01 m padding. A map, TF and laser sweep check
that entire envelope; blocking any motion component stops the whole command.
Physical mapping defaults to 0.05 m cells (`mapping_resolution`); the isolated
algorithm preview explicitly uses 0.0125 m. The global static costmap follows
the loaded map resolution; the local costmap retains its own configured resolution.
Existing saved maps retain their original resolution rather than inventing finer measurements. Unknown or
stale geometry cannot authorize motion. Planning reserves remain separate from
the physical footprint. `EE3033_ROBOT_GEOMETRY=legacy` explicitly retains the
previous 0.14 m body model for stress comparisons; simulation, oracle and all
ROS nodes record the selected profile. The DWB configuration is retained in
`config/navigation_dwb.yaml` for explicit A/B diagnostics.

Original speed is the default until the faster profile is physically validated:

| Profile | Forward cap | Angular cap |
|---|---:|---:|
| Original | 0.12 m/s | 0.40 rad/s |
| Fast trial | 0.16 m/s | 0.60 rad/s |

Apply either while mapping is stopped or before mission Start. Live changes
synchronize the gate, RPP and mission cost estimates, verify readback and
roll back on failure. An unverified rollback locks driving. The selected profile
is reused across owned stage loads in this console session; a console restart
defaults to Original unless explicitly configured. Startup Spin limits follow
the launch profile; Humble Spin caches these at configure time, so live speed
sliders do not claim to update an already configured Spin plugin.

Clearance is independent of speed. Cautious/Balanced/Bold use centre-to-boundary
reserves of 0.22/0.18/0.14 m; Bold remains the default clearance policy. Optional
gap blocking uses a 0.17 m costmap radius for the measured chassis and adds
the same 0.063199 m to the selected reserve. Without it the body radius is
0.106801 m; both include 0.01 m padding. The legacy profile retains its original
0.18/0.14 m radii and 0.04 m reserve. These are planning estimates, not
a physical passage-width guarantee.

Ordinary, observation and home actions use explicit 0.12 m / 0.20 rad,
0.10 m / 0.03 rad, and 0.12 m / unrestricted-yaw goal checkers respectively.
The active behavior trees contain no nested recovery loops. The supervisor
checks actual pose convergence and owns every recovery successor. Missing
sensor, transform or localization evidence ends navigation without attempting
recovery, including if it disappears during a reverse.

Return verification retains the assigned position tolerance and a fresh
two-second odometry window: mean absolute linear/angular speeds at most
0.035 m/s and 0.10 rad/s, XY/yaw spans at most 0.05 m and 0.12 rad, sample gaps
at most 0.25 s, and fresh zero final movement commands. Brief balance corrections
are allowed; drift, sustained turning or missing data prevent completion.

Standalone stages remain available:

```bash
ros2 launch ee3033_project mapping.launch.py allow_motion:=true
ros2 launch ee3033_project mission.launch.py \
  map:=/absolute/path/to/on-site/map.yaml target_class:=bottle allow_motion:=true
# Explicit trial profile for either stage:
# navigation_profile:=fast
```

The package reuses installed `balance_robot_base`, `balance_usb_cam` and
`lslidar_driver_n10p` drivers. It does not modify lab programs, balance PID or
existing maps. Deliberate driver reuse uses `base:=false`, `lidar:=false` and
`camera:=false`. Keep one scan adapter and one final velocity writer.

## Single-target search

The online bounded-lookahead selector ranks newly visible floor cells against travel,
heading change and observation time. Travel uses shortest paths on the
clearance-inflated grid rather than Euclidean shortcuts through obstacles.
The top three candidates receive native Nav2 path-length refinement when its
planner is available, within a 1.5-second decision budget. Timeouts retain
grid-path estimates; rejected native paths are excluded for that decision.
This remains an estimated cost, not a path-time guarantee.

Candidate generation retains all original viewpoints, then adds a bounded pass
aimed at coarse-lattice coverage holes. Planning supports cancellation and a
45-second preparation deadline. `max_candidates` is diagnostic; the explicit
`candidate_limit` rejects excessive position counts instead of silently
discarding candidates. Results distinguish unobserved and unobservable cells.

A successful navigation action alone never credits coverage. The robot must be
aligned and receive three distinct, fresh, post-arrival inference frames.
Normal empty observations can finish after 0.8 s; the existing three-second
maximum and bounded retry policy remain. The current camera geometry is used
for credit. Uncertain candidates and unsettled observations are not empty-space
evidence.

Two consistent fresh detections interrupt coverage and select a single target.
Reliable coordinates report directly after confirmation. Otherwise the mission
tries a reachable, visible 0.6–1.0 m standoff observation. Bearing-only candidates
may center and take a map-safe lateral baseline, without a blind forward
approach. After visual lock, refinement uses a 45-second work budget
of localization work, excluding travel to observation positions. Temporary loss
enters bounded local reacquisition and preserves the target lock. Exhausted
recovery alternatives or budget ends refinement without restarting coverage. No overlapping motion goals are allowed.

Finding the one target ends search immediately. Exhausting candidates, the
coverage threshold, time or goal limits does not establish target absence.
The robot returns with a NOT_FOUND result if no target was confirmed.

## Interfaces and diagnostics

| Interface | Type | Purpose |
|---|---|---|
| `/project/target_detections` | String JSON | Boxes, timestamps, class/confidence, map bearing/origin, coarse position/sigma or projection failure |
| `/project/target_estimate` | String JSON, latched | Shared map-prior posterior, covariance, estimate ID, stability and accepted freeze |
| `/project/target_observations` | String JSON | Report-quality per-frame positions, including empty frames |
| `/project/confirmed_target` | PoseWithCovarianceStamped | Confirmed map-frame position |
| `/project/perception_status` | String JSON | Inference, geometry/readiness and shared camera model |
| `/project/velocity_gate_status` | String JSON | Requested/output speeds, obstacle direction and blocking/deadline reason |
| `/project/mission_status` | String JSON | Coverage, acquisition steps, path-cost source, home/target and state |
| `/project/mission_result` | String JSON | One target report and terminal result |
| `/project/start`, `/project/stop` | Trigger | Stage-specific start/stop |
| `POST /api/navigation` | JSON `{"profile":"fast"}` or `original` | Stopped-only verified speed profile |
| `POST /api/vision` | JSON mounting/range settings | Save measured geometry for next mission load |

Target acquisition states are ACQUIRING (navigation) and LOCALIZING (stationary
confirmation). All setup/manual lockouts include these states. Stop remains
independent of parameter transactions.

## Verification

```bash
pytest -q
npm ci
npx playwright install chromium
npm run test:ui
python3 scripts/preview_ui.py   # simulated, read-only, no ROS or motors
```

An interactive **algorithm preview** runs the unchanged project console and
managed mapping/mission launches, including SLAM Toolbox, Nav2, Mapper, Mission,
YOLO perception, target filtering and the velocity gate. It does not advance
states on a timer or inject detection boxes, target coordinates or action
successes. Only the physical environment is replaced: a ray-cast lidar, ideal
planar odometry/kinematics and an approximate rendered camera containing one
photographic cup. The cup is absent during mapping and present during the
mission. Choose `cup` to search for that object; other labels are real model
labels and are not substituted into the scene. Main's parameters, speeds and
timeouts are retained, so this is not an accelerated click-through demo.

From the repository root with Docker available:

```bash
docker build -f scripts/Dockerfile.preview -t ee3033-algorithm-preview:local .
docker run --name ee3033-algorithm-preview -p 127.0.0.1:8792:8790 \
  --mount "type=bind,source=$PWD,target=/work,readonly" \
  -e PREVIEW_MAIN_COMMIT="$(git rev-parse HEAD)" ee3033-algorithm-preview:local
```

Open `http://127.0.0.1:8792/#/workflow`, start automatic mapping, wait for its
actual map save, confirm the map, set the assigned start position **and facing**
on that map, then start the autonomous mission. The simulator does not teleport
the chassis when an AMCL initial pose is entered. Stop, new trial and downloads
use the real console endpoints. Exported results are marked simulated. The
container has no device mappings or host networking; its ROS domain is isolated
from the robot. Keep it running to retain demo maps/results, and use
`docker stop -t 25 ee3033-algorithm-preview` when finished.

The simulation-only **Build a test scene** panel adds **Generate random map**,
**Random target**, and click/coordinate **Place target** controls. A uint32 random
seed reproduces a world; the target-placement seed is recorded separately.
Walls are closed, robot/target positions have clearance, and setup validates a
connected free-space route. This physical scene is never supplied as the SLAM
map or target estimate. A new world resets the simulated chassis/odometry and
loads fresh mapping in WAITING; it never starts motion automatically. Moving a
target retains the world/map and clears previous mission target evidence.
Controls reject active motion and Stop prevents a delayed stage from starting.
Scenario revisions are saved under the simulation output's `scenarios/` folder.

The camera renders **30 × 30 cm ceramic tiles**, 2.5 mm grout, lightly textured
walls and simple depth-correct shading. In simulation, **Advanced → Simulated
camera · physical setup** changes the camera itself: height, pitch, yaw, mount
translation and lens FOV update the image, TF and CameraInfo together while
stopped. FOV 0 restores the original 450 px focal length; localization reads the
matching TF/intrinsics. Minimum/maximum usable distance remain measurement
validity limits. Changes discard previous target evidence. On a real robot the
same area remains a measured-geometry configuration; it cannot move hardware.

The bootstrap downloads the official `yolo26n.pt` weights, verifies the pinned
SHA-256 and selects a cup texture from the official Ultralytics COCO128 sample.
Texture/model provenance is recorded in `/tmp/ee3033_sim/assets/provenance.json`
inside the container; the scene does not establish real camera accuracy,
balanced-chassis dynamics, motor response or actual robot performance. COCO
image rights remain with their original owners; the sample archive and source
image are identified in that provenance file. No simulated evidence should be
used as physical evaluation measurements.

Target precision is a soft objective. **Preferred precision (σ, m)** is beside
the detector class and return tolerance in mission setup (0.01–1.00 m, default
0.10 m). It is the square root of the posterior covariance's largest eigenvalue.
Changing it requires **Apply settings & reload mission** and setting the initial
pose again; it is locked during motion. The same setting reaches both the
perception close-view preference and mission acceptance policy. Valid settled observations retain their measured uncertainty even above that
value. Multi-frame consistency, freshness and map agreement still confirm the target.
If uncertainty remains above the objective after bounded A/B/C refinement
(45 seconds of localization work, excluding observation-point travel), a fresh stable estimate can be reported with
`quality: best_available`, its actual `sigma` and the preferred sigma. The UI
labels this as an approximate estimate; repeated correlated frames do not
artificially shrink its uncertainty. Missing, stale, inconsistent or geometrically
invalid observations cannot produce this fallback. Stop and navigation/obstacle
protection remain independent of the target precision objective.

The GitHub workflow also builds the Humble overlay and runs native ROS/Nav2
checks on isolated domains, including visual image/TF projection, candidate
approach/loss/Stop races, fast turning and XY-only home arrival. Neural inference
is substituted in the new perception fixture; geometry, ROS messages and TF are
real. These checks do not establish physical detection accuracy or performance.

See [RUNBOOK.md](RUNBOOK.md) for operation and [TESTING.md](TESTING.md) for the
software evidence and outstanding physical measurements.

To reproduce the hardware-free Humble suite locally:

```bash
docker build -f tests/Dockerfile.humble -t ee3033-visual-tests .
docker run --rm --network none --shm-size=512m ee3033-visual-tests
```


### Navigation fallback and repeat searches

Normal navigation remains forward-driving. Persistent failure after refreshing
costmaps and replanning can trigger one Nav2 BackUp fallback: 0.20 m at 0.04 m/s,
with an eight-second deadline. The final gate continuously requires a fresh
localized pose and known-free swept map corridor, enlarged by two-sigma pose
uncertainty, four centimetres of reserve, and cell intersection allowance. Rear
laser returns still stop the motion when available; missing rear readings alone
are not treated as proof of free space. Unknown space, stale/uncertain poses and
Stop prevent reversing. Navigation also monitors continuous odometry for blocked
translation, oscillation and nonconverging final turns; spinning alone cannot
keep a travel goal alive indefinitely. Camera calibration retains its separate
existing bounded route controller rather than inheriting the 20 cm recovery cap.

After a terminal mission, **Search again** reuses the loaded map and current
valid AMCL state without restarting ROS. It resets search/target evidence and
preserves the previous record as a separate attempt. **Reload map & set start
pose** handles invalid localization. **Use saved map** loads a reviewed map after
a console restart. Reused-map attempts are explicitly mission-only; their times
are not presented as fresh mapping-plus-search assessment times.

New trinary maps explicitly save with free/occupied thresholds 0.196/0.65 and
validate the saved image classifications. For a known legacy 0/205/254 map whose
0.25 free threshold aliases unknown pixels to free, use
`scripts/repair_legacy_map.py`: it creates an unreviewed corrected copy and leaves
the original map and assessment records untouched.


Per-frame localization decisions are published on
`/project/localization_diagnostics`. A managed mission persists them in its
assessment session under `mission_runs/mission_*/localization_frames.jsonl`.
The exported mission summary identifies that file, counted rejection reasons,
logging errors and any explicit truncation at the 8 MiB per-mission cap.
Original frame/mission timestamps, capture motion, centering, TF errors,
association, site progress, geometric conflict and MMSE acceptance are retained.

The navigation watchdog starts a separate arrival-heading window after reaching
the goal's XY tolerance; preceding travel does not consume that window. Arrival
centering counts as localization work. No remaining motion/site candidate means
holding safely and observing until the work budget, rather than ending because
five actions were sent. Preferred precision, return reserve, Stop and faults
retain their independent termination rules.


### Near-field reframing and camera occlusion

A clipped or below-minimum-range view can select up to two safe 0.18–0.30 m
observation moves away from the last observed ray (at most 0.6 m total). These
are Nav2 destinations, not unconditional reverse commands. Strict measurement
range/crop checks and the independent bearing route remain in force; a range
hint used to choose a move is never estimator evidence.

Observation candidates must be navigable and camera-visible. Visibility uses
the mounted camera origin, FOV/range, a conservative grid supercover with a
4 cm margin, and rejects unknown/out-of-map/occupied sightlines. Without trusted
depth it checks support across source-visible bearing/depth hypotheses instead
of treating a made-up one-metre point as the target. Intermediate near-field
reframing may remain below measurement range, explicitly labelled separately;
LOS, FOV and maximum range still apply. The 2-D map treats obstacles as opaque
and cannot prove visibility over low furniture or detect new unmapped occluders.


### Mapping corner recovery and capture-time TF

Translation progress requires 0.08 m over 12 s. Heading alignment must reduce
its error; rotating indefinitely does not satisfy navigation progress. Recovery
first cancels and confirms the previous action is terminal, verifies stationary
odometry and zero final commands, refreshes planning, and only then permits a
second, map-checked reverse attempt. Reverse remains capped at 0.20 m, 0.04 m/s
and 8 s, including localization uncertainty. Stop prevents every successor.
A retry never resets the outer action's absolute deadline.

Image processing holds one frame for at most one inference period when its
capture-time map transform has not arrived. It then retries that same frame,
retaining the original timestamp and normal stale-frame/TF rejection checks.
No latest-pose substitute is used for localization.

Random scenes now contain six to nine connected rooms, doorways and loops.
The generator checks physical clearance and reachability before application.
Saved scene JSON is the reproducibility record: a seed alone does not preserve
the layout across generator revisions.
