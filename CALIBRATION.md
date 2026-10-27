# Legacy camera calibration from a known map point

This optional preparation tool is retained for existing explicit profiles. The
default mission now uses camera geometry directly; no bottle collection or
validated profile is required. Open this tool from Advanced, or use the legacy
`/#/calibration` link. A profile is used only when explicitly selected.

Use **Camera setup** in the main console (`/#/calibration`). The operator marks
where the bottle base will be on a saved map, places it there and enters its
height and widest diameter. No measured distance is required. After **Start**,
the robot collects the views autonomously, fits the camera model and saves it
only if the checks pass. The earlier image-centring and manual-measurement
processes have been removed.

## Operator steps

1. Save and review a map of the current area. Select it in the map library and
   choose **Load map & camera setup**. A live mapping session must be saved first;
   camera setup will not silently substitute an older map. Fresh localization
   from the same map is reused.
2. Check the robot arrow against its actual position and heading. If necessary,
   select **Set robot pose** in the map selection tool, drag from the robot's
   location toward its heading, then press **Set robot pose on calibration map**.
   Wait for reliable localization. The system never substitutes a previous home
   pose or resets localization globally.
3. Select **Place bottle marker** and click a free floor position 0.40–1.50 m
   from the robot. Alternatively enter exact map x/y coordinates and press
   **Set bottle position**. The complete planned route must have 0.24 m map
   clearance. The dashed line previews the collection positions; the UI reports
   the required reverse extent (normally 0.80 m, up to 1.05 m when starting close).
4. Put the upright bottle's base centre **exactly at the marker**, on the same
   flat floor as the robot. Leave it fixed throughout the run. Clear the route,
   including low obstacles that lidar cannot see. Use one bottle in the scene.
5. Enter the bottle height and widest diameter in metres. The current bottle's
   defaults are **0.215 m high and 0.065 m wide**. Press **Start automatic
   calibration** and keep the page open and focused. Loading setup or marking
   the bottle does not itself move the robot.
6. The robot first makes two small opposite test turns to verify the relationship
   between angular commands and measured yaw. It then visits 15 training poses
   at five distances and three headings, followed by two held-out check poses.
   All movement follows map poses; detector box position never steers the robot.
   Two fresh complete detections are required at each capture pose. Overlapping
   proposals for the same bottle are merged; distinct ambiguous proposals cause
   capture to wait. Temporary detection loss does not change the route.
7. A passing result shows fitted pitch, camera height and both floor-check errors.
   The profile is saved automatically and reused by later missions. Stop or a
   failed run requires an explicit new Start. The same fixed map marker can be
   reused after checking that the bottle and localization remain correct.

## Geometry and limits

The fit estimates effective horizontal/vertical focal lengths, camera pitch/yaw
and mounting height. The reported 13–15° downward mounting is initialized at
14°; this is an estimate to start the fit, not a fixed pitch. Principal point and
distortion come from CameraInfo. Forward/left mount offsets come from TF
`base_footprint -> camera_link`, independently of the image header (the lab
camera uses `default_cam`, which has no mounting TF).

Every detected image is paired with timestamp-interpolated odometry transformed
into the monitored map alignment. The fit uses the actual captured map poses and
the fixed bottle coordinates, rather than assuming that commanded travel equals
actual travel. It requires at least 0.60 m of actual distance coverage and 0.20 rad
of bearing coverage. Poor or ill-conditioned fits, clipped boxes and insufficient
coverage are rejected. Both held-out floor projections must be within 0.10 m.

Gentle balancing motion is allowed during capture (0.35 s window, mean linear
speed up to 0.18 m/s and mean angular speed up to 0.60 rad/s, with bounded pose
span). Commanded motion must finish before capture. Maximum calibration commands
are 0.035 m/s and 0.12 rad/s. The existing lidar velocity gate remains the sole
final `/cmd_vel` writer. The controller stops for stale sensors/localization,
localization jumps, command-ownership loss, increasing heading error, blocked or
unknown map route, a missing browser heartbeat for 0.6 s, or a positioning/capture
timeout (40 s per pose, 300 s overall). A run is bounded to 1.2 m from its start.
Turn verification stops if the response is inconsistent or exceeds 0.18 rad.
Stationary AMCL updates are requested periodically without resetting its pose.

These checks establish consistency with the map and robot pose estimates, **not
independent surveyed accuracy**. Placement errors, map scale/localization errors,
varying balancing pitch, bottle silhouettes and floor slope can bias results.
Verify physical accuracy before claiming a measured localization error.

To use a legacy result, explicitly pass its saved path as
`calibration_file:=/absolute/path/camera_....json` when launching the assessment
console or mission. Select the **ground-plane camera** method for upright floor
objects below lidar height. Detection plus the explicitly selected geometry
projects the bottom of the object to the floor and transforms it into the map.
The calibration marker is never used as a mission target. Keep the lens settings,
mounting and floor conditions consistent, and stay within the calibrated range.
The saved schema remains compatible with existing mission perception and older
validated profiles.

## Build and launch

Use the main assessment console to select a map and manage the camera stage. It
owns the calibration child and stops that child before loading another stage.
Inside the existing robot Humble container:

```bash
bash scripts/build_robot.sh
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
source /home/ros2_ws/group4_ws/install/setup.bash
export ROS_DOMAIN_ID=6
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

The calibration launch requires `map:=/absolute/path/to/map.yaml`; motion defaults
to disabled. The main console supplies that map, its parent port, output directory
and optional fresh same-map localization seed. Standalone launches expose the
loopback HTTP API on port 8780, but the normal map-selection UI is in the main
console on port 8765. Session tokens, same-origin checks and control generations
protect writes. Do not launch a second robot stack; preflight rejects existing
command publishers and project sensor/localization nodes.

For an offline clickable simulation:

```bash
python3 scripts/simulate_calibration.py --port 8791
```

Open `http://127.0.0.1:8791`, set the simulated map point (default 0.9, 0), then
Start with the default bottle dimensions. This demo accelerates view collection
and uses the real fitter. Every output is marked simulated and is rejected by
robot profile loading. It does not test physical motion or detector accuracy.

Local checks:

```bash
pytest -q
npm run test:ui
# Source the ROS overlay first; use isolated domains, never the robot's domain 6.
ROS_DOMAIN_ID=106 ROS_LOCALHOST_ONLY=1 python3 tests/integration_calibration.py
ROS_DOMAIN_ID=108 ROS_LOCALHOST_ONLY=1 python3 tests/integration_automatic_calibration.py
```
