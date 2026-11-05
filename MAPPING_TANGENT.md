> Current release/deployment status: [README.md](README.md) and [DEPLOYMENT.md](DEPLOYMENT.md). Earlier dated status statements below are historical.

# Automatic mapping terminal heading — 2026-11-01

This local source extends the single-frame simplification baseline. Automatic
mapping uses the last nonzero segment of the validated XY path for final yaw.
Waypoint mapping retains the assigned orientation. Search, observation, approach,
and position-only return behavior are unchanged.

Mapper selects navigate_mapping.xml, whose MappingTangent planner policy is
translated to GridBased for native Navfn. The validated path supplies the final
quaternion. NavigationSupervisor requires a fresh matching endpoint and verifies
the terminal path heading before success and again after the final stationary
hold. Duplicate endpoints are ignored; a degenerate replan may reuse only the
same goal's previously computed tangent.

Validation: 982 Python tests; 16 native planner and 16 native supervisor tests.
The merged simulation source matches all 87 shared non-UI source files. On the
user's actual hand-drawn world, a 135-degree frontier request produced a
45.55-degree path tangent, with actual arrival yaw 36.12 degrees (within the
existing 0.20 rad tolerance). The quaternion matched the path segment exactly.

See validation/mapping-tangent-manifest.json for the current source fingerprint.
Full user-map simulation evidence belongs to ../EE3033_ManualSimulation_20261101/validation/.
No robot deployment or physical motion was performed by this update.

The user-field trial additionally exposed a coarse-map sample-padding deadlock
in the final motion sweep. Samples are now capped at 5 mm with the same complete
footprint, braking horizon, occupied/unknown cell squares and half-gap bound.
Regression tests and six native ROS motion-safety cases passed. No clearance or
robot geometry parameter was reduced.
