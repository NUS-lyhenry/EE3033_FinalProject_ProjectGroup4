# EE3033 — map, find and return

ROS 2 Humble software for on-site mapping, autonomous floor-target search,
map-frame position reporting and verified return to the recorded start.

## Deployed release

`manual_clearance_20261105_89a6af1ef1` adds manual navigation clearance,
a separate near-wall corner lookahead and saved map, search and path grid settings.
The [release manifest](release-manifest.json) fingerprints the 100 production
files verified on the robot.

- Adjust Nav2 planning radius, padding and inflation independently of collision protection.
- Configure the measured-profile collision protection radius from 0.07 to 0.30 m.
  Small settings also check the swept, padded wheel-inclusive body.
- Retain the effective clearances when migrating saved presets to manual settings.
- Complete automatic mapping goals by position. Explicit waypoint and observation
  headings and verified stationary return retain their checks.
- Accept a valid timestamped bottle exposure with its actual uncertainty, and
  record target reporting and return outcomes separately.

## Build and operate

Use the existing robot drivers and verified camera calibration. Build with
`scripts/build_robot.sh`, then source the resulting overlay. Launch
`ee3033_project assessment.launch.py` and open the operator console at
`http://127.0.0.1:8765/#/workflow` through an authenticated localhost SSH forward.
See [deployment](DEPLOYMENT.md), [agent instructions](agent.md), and
[bottle localization](BOTTLE_LOCALIZATION.md) before operating hardware.

Loading a stage prepares its sensors and waits for operator Start. Keep the
target absent during mapping, review the new map, establish the assigned start
pose, and then start autonomous search. Stop cancels owned work and closes motion.

## Verification

The release passed the Python regression suite, focused browser settings checks,
and isolated ROS Humble console and workflow checks. Both packages built on the
robot. The deployed assets and effective settings were read back in the stationary
`WAITING` state. See [deployment](DEPLOYMENT.md) for the current verification and
[testing](TESTING.md) for historical checks and reproduction instructions.
