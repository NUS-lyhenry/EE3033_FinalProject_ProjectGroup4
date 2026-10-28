# Frozen navigation regression corpus

These files contain complete wall geometry. Re-generating a historic seed is
not a substitute for its saved scene. `seed102_live.json` additionally preserves
the actually observed SLAM map, failed UI waypoint, path, map/odometry transform,
assigned start and stalled pose. `seed1023.json` comes verbatim from the earlier
live world; `legacy95.json` uses the original pre-upgrade generator geometry.

`random_00.json` through `random_19.json` are frozen worlds. Their acceptance
oracle independently evaluates a 0.15 m circular envelope on a conservative
2.5 cm four-connected grid of analytic wall segments. It does not call Nav2 or
the production CoveragePlanner. Unknown-map navigation must still decide using
SLAM; physical truth is not supplied to the robot algorithm.

Run only in a dedicated container, after building the current source:

```
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src --packages-up-to ee3033_project
source install/setup.bash
export ROS_DOMAIN_ID=122 ROS_LOCALHOST_ONLY=1
python3 scripts/navigation_benchmark.py \
  --case validation/navigation_cases/seed102_live.json \
  --controller rpp --map-mode known --task navigate \
  --start stalled --captured-map --output /tmp/navigation/seed102-rpp
```

Use `--controller rpp-native` for the retained unwrapped RPP control.
Use `--controller dwb` for a paired diagnostic run on the retained DWB config.
Every run records the actual action-server graph first and refuses duplicate
external servers. A diagnostic-setup failure is not an algorithmic result.

```
# Every fixed case five times, using its chosen task and map mode.
python3 scripts/navigation_benchmark.py --fixed --task navigate --map-mode known \
  --output /tmp/navigation/fixed-five
# Exactly 60 runs: 20 worlds, three repetitions respectively exercising known-map
# navigation, live-SLAM UI waypoint mapping and live-SLAM automatic map+return.
python3 scripts/navigation_benchmark.py --matrix --output /tmp/navigation/random-sixty
# Optional exhaustive 180-run cross-product.
python3 scripts/navigation_benchmark.py --matrix --full-matrix --output /tmp/navigation/full
```

`--motion delayed_deadband` introduces 80 ms command latency and physical motor
deadbands. `--motion frozen` expects a safe stopped outcome, never an arrival.
`--timeout` is an outer test watchdog, not an algorithm timeout reset.

Each directory includes the scene, launch logs, structured result, topic counts
and compressed raw timestamped diagnostic messages: plans, DWB critic
scores when enabled, map/costmaps, TF, scans, odometry, raw/final commands,
supervisor state and physical collision/actual-motion diagnostics. Negative
unreachable/frozen outcomes are separate from positive navigation successes.
All failed and interrupted runs remain in the report. A passing geometric
oracle alone is never a passing navigation run.

For three container workers, give each `--cpus 2 --network none`, a read-only
source mount and `ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=130`. Build with
`--base-paths /work/src --packages-up-to ee3033_project`. Run the commands below
in workers 0, 1 and 2 respectively (substitute their shard and output directory):

```
python3 scripts/navigation_benchmark.py --matrix --shard 0/3 \
  --output /tmp/navigation/random-worker-0 --timeout 720
python3 scripts/navigation_benchmark.py --fixed --shard 0/3 \
  --task navigate --map-mode known --output /tmp/navigation/fixed-worker-0 --timeout 240
```

Shards partition whole scenes (7/7/6 random worlds), so every worker exercises
all three modes and the slow automatic mapping runs are distributed. Each child
gets its global run index as a distinct DDS domain offset; for a 60-run matrix
with base 130 these are 130–189. The direct, current-process DDS graph must show
exactly one server endpoint per external/internal navigation and planner action.
A shard's passing result covers only its `expected_runs`; the combined 60-run
acceptance additionally requires every shard and all unique cases/repetitions.

Keep large recordings on a verified writable **host** bind mount, e.g.
`-v "$HOST_ACCEPTANCE_DIRECTORY:/acceptance"`, and write `--output` beneath
`/acceptance`. Verify a host-created marker can be read in the container before
starting. Colima's `/private/tmp` may refer to its VM rather than the Mac; using
that path without the marker check can consume the VM disk. Each worker also
uses Docker's default private IPC namespace; `--network none` and private IPC
isolate DDS endpoints from the operator's live preview, independently of the
numeric domain. Endpoint GIDs and action service owners are still recorded.

Pass `--stop-file /acceptance/STOP` to each matrix scheduler. Creating that file
interrupts the active case through its Stop/cancel path and prevents the next
case from starting. A wrapper that runs multiple phases must check the same
file before advancing to each phase. Interrupted development runs and incomplete
recordings must remain separately archived and never count as final acceptance.

Geometry profiles are explicit: `--geometry-profile measured` (default) uses the
wheel-inclusive 11.5 cm length x 18 cm width rectangle's outward-rounded 0.106801 m bounding
circle plus 0.01 m padding. `--geometry-profile legacy` retains the original
0.14 m physical circle and 0.15 m padded envelope as an independent stress test.
The environment is selected before production modules are imported and inherited
by every ROS child. The analytic oracle derives its own dimensions and refines
its grid down to 6.25 mm when a coarse grid cannot certify a connection; it never
changes the navigation map, real walls or robot dimensions. Thus the saved
28 cm old negative becomes a measured-profile positive, while the new 20 cm
case remains a conservative-circle negative. Historical `oracle` entries in
case files retain their original profile; every run recomputes its own result.

The mandatory fixed set excludes only the redundant 60 cm `narrow_doorway`
auxiliary (it remains runnable with `--case`). It includes the 36 cm positive,
the 28 cm profile-dependent case and the 20 cm measured-model negative.
For the legacy stress subset, use:

```
python3 scripts/navigation_benchmark.py --fixed --geometry-profile legacy \
  --case-ids legacy95,seed102_live,successive_right_angles,tight_passage \
  --task waypoint --map-mode known --captured-map --output /tmp/navigation/legacy-stress
```

`--start auto` is the default: seed102 and legacy95 begin at their saved original
stalled poses on every repetition. Other cases begin at their assigned start.
Use `--start origin` explicitly for a separate from-origin experiment. The
legacy95 paired fixture preserves its original SLAM grid and the previously
assigned identity map/odometry frame; its historical live TF was not captured.
