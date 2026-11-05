# RegulatedPathFollower (ROS Humble)

This plugin subclasses the installed native Regulated Pure Pursuit controller.
It keeps native path transformations, pose input, regulation and collision
checks. It does not publish odometry or motor commands.

Native RPP constrains turn acceleration relative to the angular twist supplied
by odometry. A motor input deadband can keep measured twist at zero, causing a
small first command to repeat forever. This adapter uses the preceding limited
angular command as the turn-ramp reference. Native RPP on Humble uses that
angular reference only in its rotate-to-heading acceleration constraint; linear
odometry and the actual pose/TF are passed through unchanged.

The ramp resets on Stop, lifecycle changes, a command gap over 0.5 seconds, or a
changed target position/orientation. Normal replans to the same goal preserve
it. Native collision exceptions propagate and reset the reference.

Each new plan caches sharp-bend features without changing any plan point or
collision check. Eight-centimetre direction windows suppress raster stair-step
noise; a direction change over 0.55 rad is a sharp bend. Every control tick
transforms the real pose into the unpruned plan frame at the same timestamp.
Only within 30 cm of a cached bend and within 22 cm of a raw local-costmap
lethal 254 / unknown 255 cell square, native RPP uses at most 8 cm lookahead,
0.10 rad alignment and 0.08 m/s / 0.40 rad/s ceilings. Outside that region it
restores committed settings (18 cm / 0.35 rad and the selected speed profile).
The complete plan is inspected once per update and each tick checks only
cached features, so later corners are detected without requiring a replan.
Native RPP's mutex protects cached knob changes; no parameter-server writes
occur and base_desired_linear_vel_ is unchanged. The existing uniform cap
continues to preserve curvature, including the corner ceilings. Mode changes
log effective lookahead, alignment and velocity ceilings in English. The compact
linear ceiling keeps one second of centre travel within the 8 cm carrot; the
independent final safety gate retains its full horizon and footprint.
Inflated 253 cells are excluded from this width test. The physical pose is
transformed into the local costmap frame at the same timestamp, and the raw
map mutex is released before native RPP's mutex or collision check is entered.

Humble RPP's final-heading heuristic compares carrot distance with the goal
checker's XY tolerance. A short carrot can therefore trigger final rotation
while the actual goal is far away. Before each native call, the adapter
transforms the unpruned actual goal into the physical pose frame at the same
timestamp. A short-lived checker view suppresses only that internal heading
trigger outside the real goal's XY region. ControllerServer retains the
original checker, and final braking uses it directly; all real arrival
tolerances and checker state remain unchanged. Missing/stale transforms,
nonfinite positions and unavailable/invalid tolerances fail closed.

Use plugin `ee3033_navigation::RegulatedPathFollower` with native RPP parameters
and `max_linear_vel: 0.12`, `max_angular_vel: 0.40`. The two extra caps are dynamic
positive doubles; they uniformly scale both returned velocity components to
preserve curvature. `max_angular_accel: 0.8` remains the native turn-ramp setting.
Actual chassis acceleration is a separate physical property; a command ramp
and a simulator with a physical slew limit do not validate real hardware.

`tests/integration_rpp_deadband.py` compares native and adapted controllers using
actual Nav2 `/follow_path` and the final velocity gate. Recorded evidence is in
`validation/rpp_deadband_20261028.json` and `validation/rpp_wrapper_20261028.json`.
