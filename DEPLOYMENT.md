# Robot deployment — 2026-11-01

Release: `merged_single_frame_wall_tip_20261101_513edd1751`.

The existing deployed baseline was checked before staging the integrated source.
All 93 production-file hashes, installed Python module paths, served web assets
and the active custom controller library were verified after a separate robot
build. The shared non-UI core matches the wall-tip simulation acceptance version.

The robot reached WAITING/Ready with healthy sensor/navigation status and fresh
zero final velocity. Deployment setup did not send Start, navigation goals, return
or an invented initial pose. The operator subsequently started physical mapping;
a complete physical mapping/search/return acceptance was not established by deployment.

## Installation layout

The versioned installation uses these conventional robot workspace locations:

- Source: `/home/ros2_ws/group4_releases/<release>`
- Overlay: `/home/ros2_ws/group4_workspaces/<release>/install`
- Session: `/home/ros2_ws/group4_results/assessment_<release>`
- Maps: `/home/ros2_ws/group4_maps`

`start_console.sh` records the launcher for this versioned deployment layout.
For another layout, adapt its source/overlay/session paths and verify the installed
source fingerprint. `scripts/build_robot.sh` accepts `GROUP4_WORKSPACE` for builds.
Do not overwrite live calibration or saved settings with repository defaults.

## Connection

Use the operator-approved robot SSH account and current address. Establish a
localhost-only forward from Mac port 8765 to robot port 8765, then open
`http://127.0.0.1:8765/#/workflow`. Port 8792 is reserved for the isolated local
simulation; an available web server alone does not prove a robot connection.

## Settings and recovery

Calibration, attitude, scan filter and reverse-recovery settings were preserved
byte for byte. The initial operator settings were bottle, 35–95 cm usable range,
preferred sigma 0.3 m, return tolerance 0.2 m, 0.12 m/s and 0.4 rad/s limits,
75 s ordinary goals, Bold clearance, gap blocking off and reverse recovery on.
Later UI profile changes are operator choices and require field validation.

The user authorized clearing old maps and legacy project programs. A verified
private backup was retained, along with driver dependencies and one rollback
release. Those backups, maps, network addresses and live session records are not
published in this repository.

Current configuration readback is summarized in
[validation/release-validation.json](validation/release-validation.json).
