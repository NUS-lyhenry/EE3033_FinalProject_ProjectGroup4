# Release verification — 2026-11-01

## Recorded checks

| Check | Result |
| --- | --- |
| Python regressions | 987 passed |
| Native single-frame Mission | 8 passed |
| Native moving-frame coverage | 2 passed |
| Native return-home transport | 10 passed |
| Native path planner | 17 passed |
| Robot build | Both project/navigation packages built |
| Deployment source/runtime | 93 source files, imports, web assets and loaded controller verified |

The focused native checks used synthetic sensors/actions in a localhost-only,
isolated ROS domain. They did not use the physical robot's ROS domain.
The English UI assets retained their previously validated browser behavior and
were inspected in the connected physical UI.

The manual simulation completed its unchanged field workflow with target report,
verified return and zero contact rejections. A closed field with a non-bypassable
34 cm doorway passed both directions. The native planner also passed six free
5 cm cells, which represent 30 cm. The chassis footprint was not reduced.

Physical runtime settings were read back after deployment. An operator-started
mapping run is separate from these software checks; complete physical end-to-end
acceptance and physical camera accuracy remain unclaimed.

## Reproduce software checks

Install the declared Python dependencies and run `pytest -q` from the repository
root. The frozen JSON files in `validation/navigation_cases` and the legacy
corner fixture are synthetic regression inputs, not recorded on-site maps.

For native checks, source ROS Humble and the built project overlay, set
`ROS_LOCALHOST_ONLY=1`, select an isolated domain that is not the robot's domain,
and run the focused integrations. The broader suite is available through
`tests/run_humble_checks.sh` and `.github/workflows/test.yml`.

Browser checks use `npm ci`, Playwright's Chromium installation, then
`npm run test:ui`. They use read-only local preview and intercepted fixtures.
Generated logs and browser output should remain untracked.
