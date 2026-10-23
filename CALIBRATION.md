# Camera calibration with a measured bottle

This preparation tool fits **effective focal lengths and camera mounting
pitch/yaw** from an upright bottle on a flat floor. The browser UI provides live
detector boxes, drag/numeric correction, automatic centring, bounded forward/backward
movements, measured training, independent validation, and versioned profile saving.
Mission perception uses the profile for floor projection or camera–lidar association.

Movement and fitting are automatic; you measure the dimensions and floor offsets.
Odometry limits movement and checks settling; it is never the calibration reference.

Use **Camera setup** in the main control console for the integrated one-time workflow.
The standalone launch below remains available.

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

## Guided workflow

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
Stop. Movement also requires a browser heartbeat within 0.6 seconds, fresh
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

Keep the example setup values. Use Next bottle position, Capture bottle
image, Fill reference measurement, then Record frozen measurement for positions
1–6. Fit the model, check Independent validation sample, and record positions 7–8.
The simulation uses the real fitting/validation functions, creates no ROS nodes,
and labels saved demo profiles so mission perception refuses them.

The live preview is encoded separately from detection, targeting 8 FPS at 640×480
with JPEG quality 65; frozen measurement frames use quality 85. The browser decodes
JPEG locally and retries stalled requests. Use Resume live camera to leave a frozen
view. Gentle balancing capture uses the limits described below. Fresh zero final
commands are still required; the mission return-stop verifier is unchanged.

## One-time setup in the main console

Open **Camera setup** in the main console (`/#/calibration`) and select Start
camera setup. This preparation stage owns its sensor drivers and velocity gate;
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
is removed. Check the frozen outline, measure its position, and use independent
validation to assess accuracy. This policy does not modify mission return-stop
verification.
