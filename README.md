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

## Camera-only target localization

The default `localization_method:=ground_plane` uses the visible bottom contact
of a target resting directly on a flat floor. Target depth does not use lidar.
Lidar still supplies SLAM, AMCL, navigation and obstacle protection.

No bottle fitting or multi-position collection is required. The camera model
uses matching `CameraInfo` and the image-timestamp transform from
`project_camera_optical` to `base_footprint`. Full TF rotation includes pitch
when the installed driver supplies it. Keep the real camera mount/optical axes
correct; a mathematically valid default CameraInfo is not proof of accuracy.

The map also shows an unconfirmed live target estimate before mission reporting.
Recent finite detections are weighted by inverse variance and age over an 8 s
window. The dashed circle shows one-sigma uncertainty including sensor error
and observed scatter; correlated camera frames do not artificially eliminate
the uncertainty floor. Lost estimates fade and expire after 30 s. These display
estimates never start navigation or bypass mission confirmation. **Measure
distance** supports two clicks or a drag on the map, without sending pose or
motion commands.

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
centered target can use the remaining bounded acquisition time. Unresolved
visual viewpoints get one deferred retry after other viewpoints.

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

Automatic frontier selection, deferred-frontier retries, return and unique map
saving retain the existing mapper algorithm. Occupied/unknown space still
bounds reachable goals. The rotation shim/DWB controller now tolerates modest
path-heading changes before choosing an in-place turn (0.70 rad entry,
0.35 rad exit). Heading-aware progress, bounded recovery, footprints, lidar
self-mask, obstacle distances, command deadlines and heartbeat checks remain.

Original speed is the default until the faster profile is physically validated:

| Profile | Forward cap | Angular cap | Shim turn speed |
|---|---:|---:|---:|
| Original | 0.12 m/s | 0.40 rad/s | 0.35 rad/s |
| Fast trial | 0.16 m/s | 0.60 rad/s | 0.55 rad/s |

Apply either while mapping is stopped or before mission Start. Live changes
synchronize the gate, DWB/shim and mission cost estimates, verify readback and
roll back on failure. An unverified rollback locks driving. The selected profile
is reused across owned stage loads in this console session; a console restart
defaults to Original unless explicitly configured. Startup Spin limits follow
the launch profile; Humble Spin caches these at configure time, so live speed
sliders do not claim to update an already configured Spin plugin.

Clearance is independent of speed. Cautious/Balanced/Bold use centre-to-boundary
reserves of 0.22/0.18/0.14 m; Bold remains the default clearance policy. Optional
gap blocking adds 0.04 m reserve and uses a 0.18 m costmap radius. Without it the
radius is 0.14 m; both include 0.01 m padding. These are planning estimates, not
a physical passage-width guarantee.

Home uses an XY-only goal checker, avoiding an unnecessary final heading turn.
Search viewpoints retain heading requirements. If `return_heading_required`
is explicitly enabled, home uses the normal heading-aware checker instead.

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

The online greedy selector ranks newly visible floor cells against travel,
heading change and observation time. Travel uses shortest paths on the
clearance-inflated grid rather than Euclidean shortcuts through obstacles.
The top three candidates receive native Nav2 path-length refinement when its
planner is available, within a three-second decision budget. Timeouts retain
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
may reorient in place but do not command a blind forward approach. Acquisition
is bounded to three goals and fifteen seconds, including movement. Losing the
candidate or exhausting the budget resumes coverage after confirmed
cancellation. No overlapping motion goals are allowed.

Finding the one target ends search immediately. Exhausting candidates, the
coverage threshold, time or goal limits does not establish target absence.
The robot returns with a NOT_FOUND result if no target was confirmed.

## Interfaces and diagnostics

| Interface | Type | Purpose |
|---|---|---|
| `/project/target_detections` | String JSON | Boxes, timestamps, class/confidence, map bearing/origin, coarse position/sigma or projection failure |
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

The bootstrap downloads the official `yolo26n.pt` weights, verifies the pinned
SHA-256 and selects a cup texture from the official Ultralytics COCO128 sample.
Texture/model provenance is recorded in `/tmp/ee3033_sim/assets/provenance.json`
inside the container; the scene does not establish real camera accuracy,
balanced-chassis dynamics, motor response or actual robot performance. COCO
image rights remain with their original owners; the sample archive and source
image are identified in that provenance file. No simulated evidence should be
used as physical evaluation measurements.

Target precision is a soft objective. The default preferred sigma is 0.10 m;
valid settled observations retain their measured uncertainty even above that
value. Multi-frame association and spatial consistency still confirm the target.
If precision remains below the objective after at most three observation moves
or 15 seconds of refinement, a fresh stable estimate can be reported with
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
