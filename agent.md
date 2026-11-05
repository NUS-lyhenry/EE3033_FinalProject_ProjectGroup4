# EE3033 project instructions

## English-only UI

All operator-facing interface text MUST be English: headings, labels, buttons,
menus, instructions, status/error messages, tooltips, accessibility text and
canvas text. Conversation and internal documentation may use another language.
Inspect both static and dynamically generated text before delivering UI changes.

## Release ownership

This branch records `manual_clearance_20261105_89a6af1ef1`.
Read README.md, DEPLOYMENT.md, TESTING.md and release-manifest.json first.
Preserve the recorded source fingerprints when reproducing the deployed version.
Do not equate builds, simulation, GitHub pushes or a connected UI with completed
physical acceptance. State the verification scope accurately.

## Hardware and operator work

Inspect the current service and task before changing robot processes. Preserve
operator-started work. Starting a service, loading a stage and commanding motion
are distinct actions; never infer movement authorization from a read-only request.
Never inject a guessed robot pose or target position. Keep full-footprint,
collision, freshness, cancellation and stationary checks intact.

Preserve calibration, driver dependencies, maps and settings unless the user
explicitly authorizes a change. Use a separate versioned overlay and retain a
rollback path for deployment. Read actual runtime settings after activation.

## Development

Keep uncommitted work intact; use an isolated checkout for release preparation.
Use localhost-only isolated ROS domains for synthetic/native tests, never the
physical robot's domain. Run checks appropriate to the change.
Keep credentials out of commands, scripts, commits and logs. Do not commit real
maps, live telemetry, private backups or generated test/build output.
