# Camera calibration with a measured bottle

The default workflow needs **bottle height, widest diameter, and one measured
floor distance**. In a mapped area, the robot automatically centres a fixed bottle,
collects images at different poses, fits effective camera geometry and checks two
held-out positions. A passing profile is saved and reused by mission perception.

Use **Camera setup** in the main console (`/#/calibration`). Select the area's
reviewed map in the map library and load camera setup. Fresh localization on that
same map is reused. If localization is unavailable, establish the robot's current
pose using the map gesture or the optional localization fields. No previous home
pose is silently reused. Loading this stage starts map server and AMCL alongside
its own sensor drivers and velocity gate; it does not start a navigation mission.

## Automatic workflow: one measurement

1. Put one upright bottle on the same flat floor as the robot, ahead of the
   camera. Its whole outline and floor contact must be visible. Clear at least
   0.78 m behind the robot, with 0.24 m clearance on each side, including low
   obstacles below the lidar plane. Keep the bottle fixed throughout collection.
2. Enter its height and widest diameter in metres. Measure **one straight floor
   distance**, from directly below the camera lens to the centre of the bottle
   base at the robot's current position. It can be diagonal. Do not measure a
   slanted lens-to-bottle distance or distance to the bottle's nearest surface.
3. Select **Start automatic calibration** immediately after measuring. The robot
   centres the bottle, reverses up to 0.72 m, collects views at five distances and
   small yaw offsets, then moves partway forward to two new check positions.
   Keep the page open and focused; Stop is available throughout. No further
   dimensions, offsets, box selections or distance measurements are requested.
4. The program fits focal lengths, pitch/yaw, camera height and the bottle's
   initial bearing together. The reported 13–15° downward mount is initialized
   at 14°; this is a starting estimate, not a fixed calibration or angle limit.
   The result shows the fitted pitch in degrees. It uses the installed TF for lens forward/left
   offsets and CameraInfo for principal point and distortion. The physical
   mount frame defaults to `camera_link`, independently of the image header
   (`default_cam` on the lab camera has no mounting TF). It pairs each image
   with timestamped interpolated odometry, using map localization to check the
   route and detect localization shifts. It never assumes that commanded travel
   equals actual travel.
5. Two images from held-out translation positions check the projected floor
   location. Both must have error at most 0.10 m against pose-derived positions;
   clipped boxes, insufficient motion/angle coverage, poor fits or failed checks
   prevent saving. A passing profile is saved automatically and shown in the UI.

These checks measure consistency with the robot's pose estimates, **not independent
surveyed accuracy**. Odometry bias, detector outlines, varying balancing pitch or
an inaccurate initial distance can still bias localization. Verify physical
accuracy separately before claiming a measured target error. The calibration
bottle's inferred position is never used as a mission target coordinate.

Automatic movement is bounded to a short straight corridor, 0.035 m/s translation,
0.15 rad/s rotation and a 210-second overall deadline. Unknown/occupied map cells,
localization jumps or stale localization, missing/ambiguous target detections,
ownership loss stops collection. A missing browser heartbeat disables driving
after 0.6 seconds; automatic collection ends if it remains missing for 1.5 seconds. A brief sensor or
detection delay commands zero immediately and waits for fresh, settled data; a
delay lasting over two seconds stops the run. Live lidar
stopping remains the final velocity gate. A stopped/failed run requires a new
measurement at the robot's current position and an explicit new Start.

The **Advanced** panels retain the manually measured workflow below for cases
without a saved map or when independent reference measurements are needed.

## Start the preparation session

Build the updated package with `bash scripts/build_robot.sh`, then source its
separate overlay. Stop the assessment, mapping, mission and earlier follower
launches first. Calibration refuses existing robot command publishers and never
stops another stack for you. Inside the robot's existing Humble container:

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
source /home/ros2_ws/group4_ws/install/setup.bash
export ROS_DOMAIN_ID=6
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch ee3033_project calibration.launch.py allow_motion:=true \
  map:=/home/ros2_ws/group4_maps/YOUR_AREA/map.yaml \
  output_directory:=/home/ros2_ws/group4_calibrations
```

Forward local port 8780 to the robot's loopback port 8780 using the same SSH
connection used for the assessment console. Open **http://127.0.0.1:8780**.
If the container uses a separate network namespace, expose/forward its loopback
port in the same way as assessment port 8765. The server binds only to loopback
and requires its session token for controls.

Motion defaults to disabled. Omit `allow_motion:=true` to reposition the bottle
manually and collect stationary measurements. Deliberately reused lab drivers can
be selected with `base:=false`, `lidar:=false`, or `camera:=false`; calibration
still owns its one scan adapter and final velocity gate. The detector defaults
to `bottle`, with the existing `/home/ros2_ws/weights/yolo26n.pt` model.

## Advanced manually measured workflow

1. **Measure and save setup.** Enter the bottle height, widest diameter, lens
   height above the floor, and lens forward/left offsets from `base_footprint`, in
   metres. Replace the example defaults with physical measurements. Keep the
   robot upright, floor level, and lens settings fixed.
2. **Position the target.** Keep exactly one bottle fully visible. Centre turns
   toward its image centre. Forward/backward controls request one movement of at
   most 10 cm, with limits of 0.035 m/s and 0.15 rad/s. Translation times out after
   5 seconds; centring after 8 seconds or 0.45 rad. The existing lidar gate stays
   active, so a blocked move may end before reaching 10 cm.
3. **Capture and measure.** Keep the positioning controls idle. Gentle balancing
   movement is allowed. Capture a fresh bottle image, then correct its box if
   necessary. Include the top and floor contact, excluding the shadow. Measure
   from the floor point directly below the lens to the **centre of the bottle
   base**: forward distance parallel to robot forward, and left offset perpendicular
   to forward. Left is positive; right is negative. These are two floor components,
   not a diagonal, bumper distance, nearest-surface distance, or slanted lens
   distance. Remeasure both after turning.
4. **Train at six or more positions.** Cover at least 0.60 m of forward-distance
   span and 0.20 rad (about 12°) of left/right bearing span. For example, use
   forward distances 0.65, 0.8, 1.0, 1.15, 1.35, 1.5 m with measured side offsets
   around ±0.1–0.2 m. Adapt these to keep the whole bottle visible. Move the
   bottle sideways if needed and record each frozen measurement once.
5. **Fit, then check two new positions.** Enable “Independent validation sample”.
   Capture two positions within the training distance/bearing range, each at least
   8 cm from every previous position. Both projected floor positions must pass
   a **0.10 m** error limit against your measurements. Failed checks stay visible
   and block saving; correct an incorrect measurement or retrain and refit.
6. **Save the validated profile.** Every save creates a new JSON file, preserving
   earlier profiles. The UI provides the exact assessment launch command. Ctrl+C
   the calibration launch before starting the assessment stack.

Stop remains available throughout. Hiding/closing the page or losing focus requests
Stop. Manual movement requires a browser heartbeat within 0.6 seconds; automatic
movement also commands zero after 0.6 seconds and ends the run after 1.5 seconds.
All movement requires fresh
image/detection, lidar, odometry and final output, plus exclusive command ownership.
Calibration never writes directly to `/cmd_vel`.

## Use the result autonomously

Use the actual saved path shown by the UI:

```bash
ros2 launch ee3033_project assessment.launch.py allow_motion:=true \
  calibration_file:=/home/ros2_ws/group4_calibrations/camera_SAVED_PROFILE.json
```

The console forwards that profile into each mission-stage launch. Choose
**Camera + floor plane** for targets whose visible bottom rests on the same floor.
Perception projects that contact pixel to `base_footprint`, then transforms it to
`map` at the image timestamp. Observations outside the trained distance/bearing
envelope are rejected. Confirmation and target filtering remain active.

**Camera + lidar** uses the corrected intrinsics and measured camera mount when
associating scan points with detections. The target must still intersect the
scan plane. It reports the associated visible surface, which can differ from the
object's centre. This tool does not calibrate the lidar mount or eliminate
ambiguous wall/background associations.

Standalone `mission.launch.py` accepts the same `calibration_file` argument. With
no profile, the existing CameraInfo/TF behaviour remains. An invalid or unvalidated
profile prevents perception startup; camera resolution/frame mismatch rejects
observations. Saving does not alter a running mission, publish replacement TF,
or modify lab camera files, balance PID, or saved maps.

## Limits

A detector box approximates a silhouette. The fit uses a vertical centreline and
known mid-height diameter; irregular shape, a narrow neck, occlusion or loose box
can bias it. Principal point and lens distortion are retained from CameraInfo.
Missing intrinsics are explicitly labelled “assumed image centre / zero distortion”.
The tool **does not estimate a full distortion model or camera roll**. A checkerboard
is better for full intrinsic calibration; the underlying projection follows the
[OpenCV camera model](https://docs.opencv.org/4.12.0/d9/d0c/group__calib3d.html).

Balancing pitch changes, floor slope, altered focus/resolution, mount movement
and measurement error can invalidate the result. Relative floor validation does
not establish map accuracy: mapping, robot localization and TF also contribute.
Use independent map reference positions before claiming physical target accuracy.

The briefing permits prepared calibration data. Run this tool before the assessed
workflow, and remove the bottle before assessment mapping. The mission uses
camera parameters, never a training sample's target position. This tool is separate
from stage 2, where manual target coordinates and driving are prohibited.

For a simulated, read-only preview without ROS:

```bash
python3 scripts/preview_ui.py
# http://127.0.0.1:8790/calibration
```

For a **clickable simulation** of the complete workflow:

```bash
python3 scripts/simulate_calibration.py
# http://127.0.0.1:8791
```

For automatic simulation, keep the example bottle height 0.25 m, diameter
0.065 m and distance 0.90 m, then select Start automatic calibration. The
accelerated collection finishes in about six seconds. The Advanced panels also
support the manually measured simulation with reference positions 1–8.
The simulation uses the real fitting/validation functions, creates no ROS nodes,
and labels saved demo profiles so mission perception refuses them.

The live preview is encoded separately from detection, targeting 8 FPS at 640×480
with JPEG quality 65; frozen measurement frames use quality 85. The browser decodes
JPEG locally and retries stalled requests. Use Resume live camera to leave a frozen
view. Gentle balancing capture uses the limits described below. Fresh zero final
commands are still required; the mission return-stop verifier is unchanged.

## One-time setup in the main console

Open **Camera setup** in the main console (`/#/calibration`), select a reviewed
map, and select Load map & camera setup. This preparation stage owns its sensor drivers and velocity gate;
loading mapping or a mission first stops that stage. Calibration does not start
the assessment timer. The main Stop button also stops camera positioning.

Save a validated profile once. The console discovers validated profiles in its
`calibration_directory` (default `~/ee3033_calibrations`) after each restart and
forwards the latest valid profile into mission perception. Invalid and simulated
profiles are ignored. Camera-only calibrated floor projection becomes the default
unless the operator explicitly chooses another method. Recalibrate if the mount,
lens settings or resolution changes. The embedded panel needs both console8765
and calibration8780 forwarded through SSH.

Calibration capture now permits bounded balancing motion over a 0.35 second window
(mean linear speed<= 0.18 m/s, mean yaw rate<= 0.60 rad/s, XY span <= 0.15 m,
heading span <= 0.45 rad). It still requires fresh sensors, zero final movement
commands and idle positioning controls. One fresh detected image is captured;
the image and its box remain paired. The earlier multi-frame outline-jitter gate
is removed. Automatic collection uses the same balancing tolerance. The advanced manual
workflow also supports frozen-outline corrections and independently measured checks. This policy does not modify mission return-stop
verification.
