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
it. Native collision exceptions propagate and reset the reference. There is no
adaptive lookahead or collision-check bypass.

Use plugin `ee3033_navigation::RegulatedPathFollower` with native RPP parameters
and `max_linear_vel: 0.12`, `max_angular_vel: 0.40`. The two extra caps are dynamic
positive doubles; they uniformly scale both returned velocity components to
preserve curvature. `max_angular_accel: 0.8` remains the native turn-ramp setting.
Actual chassis acceleration is a separate physical property; a command ramp
and a simulator with a physical slew limit do not validate real hardware.

`tests/integration_rpp_deadband.py` compares native and adapted controllers using
actual Nav2 `/follow_path` and the final velocity gate. Recorded evidence is in
`validation/rpp_deadband_20261028.json` and `validation/rpp_wrapper_20261028.json`.
