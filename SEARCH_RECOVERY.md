> Current release/deployment status: [README.md](README.md) and [DEPLOYMENT.md](DEPLOYMENT.md). Earlier dated status statements below are historical.

# Search recovery and station observation

The earlier search-recovery release was deployed; this current local
`single_frame_simplification_20261101` revision is **not deployed or physically
validated**. [DEPLOYMENT.md](DEPLOYMENT.md) and `release-manifest.json` preserve the
old receipt rather than claiming these changed files are on the robot. Current
software checks belong in [TESTING.md](TESTING.md).

## Search behavior

Search uses map-derived camera visibility, safe path costs and adaptive
three-station lookahead. Each predicted station action groups useful headings
and charges translation once. At the current physical station (within 0.15 m),
turn cost uses the direct yaw change. The local heading batch is preferred when
its new-coverage-per-time score reaches 85% of the best global station batch. This keeps
high-value views together without requiring a full rotation at every waypoint.
At most six candidate headings are attempted at a station; both successful and
failed attempts consume a slot, while correction of the same heading does not
consume another. The station also has a 25-second budget, starting on arrival
rather than departure for that station. Turns and observation retries share that
residence budget. Each search turn has a 12-second deadline and 0.20 rad heading tolerance, without navigation recovery
or backup for a failed search turn. A failed heading remains unresolved. Target detection interrupts
the sweep through the same cancellation path as ordinary search navigation.

Coverage is credited from each fresh, ready exposure using its actual
capture-time camera model and map pose. Normal travel, balancing and turning are
allowed; neither goal arrival, a stationary window, three-frame collection nor
OBSERVING state is required. Source time, model and pose must agree, the exposure
must be within 1.5 seconds and the current epoch. Footprints are unioned per actual
frame, never interpolated between frames. Target-containing frames also count as
viewed: observed coverage is not empty-space evidence. Physical stations group
nearby exposure poses for audit and bounded revisit behavior.

Routine candidate ranking no longer requests three native Nav2 paths per
selection. Safe-grid costs still rank stations; the selected goal retains full
Nav2 planner/footprint checks, and disconnected-region probes remain separate.

Previously departed search stations receive a five-second revisit cost when
choosing another position; they are not permanently banned. Search
candidates within 0.20 m of a remembered failed localization site are excluded. These are selection policies; they do
not bypass map, visibility, collision, localization or motion-permission checks.

## Loss, stall and return to search

A target still needs two consistent fresh associated detections to lock. Local
reacquisition uses a ten-second loss timeout. The timeout tracks the last fresh
associated usable bearing, rather than inference messages that do not match the
locked target. The bearing must come from a new exposure, pass bearing validity
and freshness checks, and belong to a sequence with at least two consistent
associated detections. Duplicate exposure timestamps cannot refresh this timer.
Successful reacquisition refreshes it.

If detection continues but localization makes no useful progress for ten seconds,
the mission first tries one safe change of observation position. A further stall,
or no usable alternative, can request a restart of map search. Progress means
sampling counts exceeding their current high-water mark: the calibrated bottle
batch-frame count, or the total accepted bearing-site frame count for other
modes. A batch repeatedly resetting at the same frame count cannot keep the
watchdog alive; inference traffic and repeated estimates do not count. A new
observation site starts a new sampling high-water mark. The stall observation clock pauses while traveling
to an observation site; the loss timeout and per-acquisition wall clock continue.

Before restarting, the owner closes motion permission, cancels active or pending
navigation and waits for its terminal result. Stop, explicit return, faults,
mission limits and return-time reserve take precedence. Restarting does not
change the recorded home or reset the mission clock, goal count or return reserve.
An interrupted in-progress search attempt is counted once toward the 120-attempt
limit; a lock acquired while selecting the next point does not count the previous
completed observation again. Each new target acquisition has its own 90-second
wall-clock window within the unchanged overall mission budget. At most two
automatic map-search restarts are allowed per mission.

The last two successfully observed physical stations have their recorded visible
footprints removed from the observed set. Their headings are eligible for another
attempt, including headings previously deferred only because visual evidence was
unresolved. Older coverage and actual navigation failures remain recorded. With
fewer than two valid stations, only the available valid footprints are reopened;
an unsuccessful view never creates coverage credit.

The old lock, association, sampling/estimate cache and refinement state are
cleared for the restarted acquisition. Its `search_epoch` and exposure boundary
exclude old frames and delayed results. The owner enters `RESETTING` and waits
up to three seconds for an empty same-epoch target-filter acknowledgement before
selecting another search station. A missing acknowledgement stops with an
explicit reset-acknowledgement error (`state: STOPPED`, with `internal_error` and
`termination_reason`). Historical detection evidence and restart events remain
in the exported mission record. A discarded estimate cannot become the newly
confirmed target merely by arriving late.

When recovery limits or remaining time prevent further search, the existing
report/return policy applies. A usable reported coordinate can produce COMPLETE
after verified return. A detected target without a usable coordinate produces
FOUND_UNLOCALIZED; incomplete search evidence remains SEARCH_INCOMPLETE.
NOT_FOUND requires complete coverage evidence and no confirmed target evidence;
manual early return cannot establish target absence.

## Defaults and unchanged budgets

| Setting | Default / behavior |
|---|---|
| `reacquisition_timeout` | 10 s since a fresh associated bearing |
| `localization_stall_timeout` | 10 s collecting without progress; one safe position change before restart |
| `max_search_restarts` | 2 per mission |
| `filter_reset_timeout` | 3 s for same-epoch empty estimate acknowledgement |
| Coverage reopened | Last 2 valid physical station footprints |
| Failed localization-site exclusion | 0.20 m |
| Completed search-station revisit cost | 5 s |
| Minimum local station-batch score | 0.85 × best global station-batch score |
| Station observation limit | 6 attempted candidate headings / 25 s; same-heading correction shares its slot |
| Individual search-turn limit | 12 s / 0.20 rad heading tolerance |
| `acquisition_seconds` | 90 s wall clock per acquisition, including travel and local recovery |
| Candidate preparation deadline | 45 s; distinct from target acquisition |
| Per-decision planning budget | 1.5 s |
| Total mission limit | 900 s with an independent return reserve |

Bottle distance still uses the fixed 17-point model and defaults to a 35–95 cm
discovery range. One valid associated exposure is accepted with its true source
timestamp, uncertainty and `quality: approximate`. Normal motion is allowed;
fixed observation waits, an eight-frame requirement and movements solely to
improve an already valid coordinate are removed. At most one useful centering
turn is allowed (2.5 seconds; ±32 px normalized to 640 px width), followed by final
report and return. Invalid coordinates can still trigger bounded recovery.

The accepted record uses `source: bottom_inverse_single`, `source_frames` with one
real exposure, `frames/samples: 1` and `measurement_stable: false`. Acceptance,
reporting and measurement stability are different facts. Console serialization
and frozen estimates must preserve that distinction. Map validity, calibration,
source freshness at acceptance, epoch and exposure-time pose checks remain.
This change does not establish physical positioning accuracy or repair an unsafe
return start pose.

Return verification is a position-only 0.5-second hold within the existing home
tolerance, XY span at most 0.05 m, fresh odometry and fresh zero final commands.
It does not test instantaneous speed or yaw. Observation has no stationary gate.

## Status and compatibility

Mission status adds `search_epoch`, `epoch_started_stamp`, `search_restarts`,
`search_restart_history`, `target_ever_detected`, `failed_localization_positions`,
`observation_history`, `station_views`, `station_attempts`, `viewpoints_attempted`
and `pending_action`. `station_views` counts credited observations, while
`station_attempts` enforces the candidate-heading attempt cap. `target_reported` distinguishes an accepted `target` record (false) from a final
coordinate report (true). Missing `target_reported` remains compatible with old
records where non-null target meant reported. `target_detected` describes the current
lock; `target_ever_detected` preserves earlier confirmed detection across resets.
`pending_action: RESEARCH` allows operator Return to replace a pending restart
while cancellation is finishing; it cannot override a pending Stop. RESETTING
remains an active, setup-locked state in the owner, backend and UI. Restart
history records its reason, invalidated station/footprint count and reopened
options. Full station footprints and observation/departure/failure events are
saved in the mission's `search_observations.json`; status contains a compact
history and recent events rather than repeating all cell coordinates. Filter estimates/diagnostics carry the same epoch; epoch zero
retains initial-mission compatibility, while old or missing epochs after a
restart are not accepted. These fields extend existing status/result messages;
search restart is an internal owner transition, not a new manual-motion API.

## Verification and deployment status

Software acceptance covers loss-timeout recovery, stall progress/reset behavior,
one relocation before restart, the two-restart cap, late navigation/estimate
callbacks, Stop/return precedence, coverage removal/reopening, same-station
heading selection and bounded turning. Existing target association, bottle fusion,
search completeness and home-verification checks must continue to pass.

### Reproducible evidence

- [Recent-search input](validation/recent_search_input.json.gz) is a read-only
  snapshot of three recorded physical searches, with retrieval provenance and
  uncompressed JSON SHA-256 in [its manifest](validation/recent_search_input_provenance.json).
- [Earlier watchdog decision replay](validation/recent_search_watchdog_replay.json)
  applied the search-recovery baseline watchdog methods to those diagnostics. It checks
  which decision would occur first: loss-timeout restart for two records and an
  attempted safe alternate site for the stalled third record. It does not
  reconstruct camera headings, prove an alternate site is feasible, or replay
  navigation/physical motion.
- [Earlier station planner comparison](validation/station_search_benchmark.json)
  compares identical maps, an analytic camera footprint and ideal fresh frames,
  using the [bundled unmodified baseline](validation/historical/search_baseline_coverage.py).
  [Historical provenance](validation/historical/provenance.json) records the
  baseline source and hash. These are estimated planner costs, without YOLO,
  Nav2 execution or hardware dynamics.

From the project directory, reproduce without a robot connection:

```bash
PYTHONPATH=src/ee3033_project python3 scripts/replay_search_watchdogs.py validation/recent_search_input.json.gz
PYTHONPATH=src/ee3033_project python3 scripts/benchmark_station_search.py
```

This update has no new physical search performance measurements. After software
checks, deployment requires the usual build, effective-file/parameter readback
and UI connection checks. Physical validation should compare a narrow passage,
a corner requiring multiple headings, intermittent target loss and a target
visible but not localizable, recording station revisits, travel/turn time,
coverage, restart reasons and final outcome. Planner or mocked-transport results
cannot substitute for those measurements.
