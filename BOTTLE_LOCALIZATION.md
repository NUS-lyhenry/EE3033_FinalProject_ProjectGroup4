> Current release/deployment status: [README.md](README.md) and [DEPLOYMENT.md](DEPLOYMENT.md). Earlier dated status statements below are historical.

# Current local bottle policy — 2026-11-01

Local version `single_frame_simplification_20261101` is **not deployed or
physically validated**. The dated release notes below retain the previous
8-frame/10-pixel policy and its historical evidence; they do not specify this
new local behavior. See [README.md](README.md) and [SEARCH_RECOVERY.md](SEARCH_RECOVERY.md).

The fixed 17-point inverse model, original-image geometry, range limits and
exposure-time map transform remain. A single valid associated exposure can now
supply an approximate coordinate, including during normal driving, turning and
balancing motion. Actual covariance/sigma and quality flags are retained. No
stationary observation window, fixed 1.5/2-second improvement wait or eight-frame
batch is required. New frames may update the evidence; their count is never
invented or used to manufacture multi-frame stability.

The first accepted result is retained in `mission.target` with
`target_reported: false`. The owner cancels search, optionally turns once toward
preferred centering (±32 px normalized to 640 px width, at most 2.5 seconds), then
finalizes `target_reported: true` and returns. A valid coordinate does not require
extra movement to 60–80 cm or another view solely for precision. Stop preserves
the accepted record and stops motion; it does not initiate return.

A single-frame result keeps `source: bottom_inverse_single`, its one real source
stamp, `frames/samples: 1`, `measurement_stable: false`, actual observation age,
and `quality: approximate`. The result display independently identifies target
detection, accepted/preview versus reported coordinates, and return verification.
A reported record stays visible after a return failure; a preview never becomes
a report merely because a map marker exists. No physical accuracy guarantee is
inferred from its sigma or a successful software check.

---

> Historical release note. Deployment IDs, state snapshots and validation counts below describe that release. Current source behavior and deployment status are in [README.md](README.md) and [SEARCH_RECOVERY.md](SEARCH_RECOVERY.md).

# Bottle localization — 2026-10-31

This release is based on the local `EE3033_Radius10_20261030` source. It is deployed as `inverse_range_20261031_5890a5a5ee`, with file fingerprints and live bottle/range settings verified. The connected UI is waiting for the assigned initial pose; no physical movement test started. The new path is automatic for `target_class:=bottle`; saved legacy ground-plane, hybrid, pitch-override and controller-reference settings do not change the bottle model. Other object classes retain the legacy localization modes.

## Ranging and discovery

For the original, unrectified 640×480 image, `d_m = 214.25159 / y_bottom - 0.1282935`. The distance is from the wheel axle to the bottle base centre. No bottle radius, optical-frame translation, or camera pitch compensation is added. Capture-time `map <- base_footprint` supplies the axle pose once. Matching original CameraInfo, collection weights and YOLO inference size 416 are checked. Detector boxes must be in original-image coordinates.

The exact 17-point bottom-y interval is 188.70762634277344–445.91217041015625 px. Replay of the supplied coefficients gives RMSE **0.507922 cm**; re-fitting each leave-one-out subset gives **0.564935 cm**. This is a reproduction of the same dataset, not a new physical validation. Dataset identity, intrinsics, model hash, inputs and replay errors are in [archived calibration provenance](validation/historical/calibration-provenance.json).

The camera detection/coverage range defaults to **35–95 cm**, with both UI and backend enforcing **35 ≤ minimum < maximum ≤ 100 cm**. Mission settings pass `bottle_min_range` / `bottle_max_range` through the launch file to perception. Apply settings and reload the mission before starting. Coverage and camera-grid visibility use these values. A YOLO box beyond the selected maximum remains annotated but does not trigger target acquisition, count as an eligible detected candidate, or provide a coordinate measurement. The fitted estimate must also be within the selected maximum, even at an approximately rounded calibration endpoint. Normal map support remains within the selected range along the observed ray.

## Bounded efforts to improve the view

Prefer the original image midline ±10 px and an observation position around 60–80 cm. Try at most **one** centering correction at a station, with a **2.5 s** motion budget. After **1.5 s** without preferred centering/posture, stopped observations can be fused with increased uncertainty. Off-centre pixels incur an explicit uncertainty penalty and an approximate-quality flag. There is no hard high-precision centering requirement. An alternate safe observation location may be chosen after a short observation window, with at most three sites in total. Navigation clearance and collision checks are retained.

The controller's fresh-packet roll/pitch stream checks capture-time oscillation only. The old tilt zero, camera-angle fitting, wheel-pivot calibration and pitch override are not used for bottle distance. Stable telemetry cannot establish that a changed fixed mount matches the collection. If telemetry is unavailable or oscillating, a stopped observation may continue after the short improvement window with an added 5 cm engineering uncertainty term. This is not an attitude correction or a validated error bound. Actual chassis translation/yaw must still pass the stopped-motion window; missing localization, invalid timestamps and invalid image geometry are not relaxed.

## Close bottles and bottom clipping

Here “bottom at zero” means the distance from the image's lower edge is zero: original-image `y_bottom ≈ 480`. A valid close box below 35 cm, or a box clipped at the lower image edge, uses a **near-field guess**, never inverse-model extrapolation or clamping. The initial axle range prior is **12–35 cm** (midpoint 23.5 cm), combined with horizontal bearing and admissible map support. These bounds are engineering assumptions, not additional calibration data. The guess retains at least **12 cm** reported positional uncertainty, carries `near_field_guess` / `approximate` flags, and cannot gain precision merely by repeating frames.

The robot first obtains a stopped observation. It may try a bounded map-safe retreat when useful. If the route is narrow and no safe alternative is available, it can report a fresh, stable approximate batch and return; it does not advance into a near bottle or wait until the acquisition timeout merely to improve precision. A truly invalid `y=0`, tiny box, missing contact metadata or horizontally clipped bearing is not substituted into the inverse formula.

## Fusion and acceptance

Each observation group requires **8 distinct, fresh, associated exposures** at one stopped site. Each image is transformed with its own capture-time TF and recent capture-time localization covariance. Median/MAD rejection and median fusion remove jumps. The group retains frame scatter, the largest per-frame covariance, pose/yaw effects and the shared error floor. A new rolling group replaces the previous correlated likelihood. Neither legacy floor projection nor bearing intersection is added as a second independent measurement from these images.

The normal visual engineering floor is 2 cm, with larger terms for pixel sensitivity, motion, localization, posture and off-centre observations. Near guesses use the larger 12 cm floor. Neither number is a statistical accuracy guarantee. Map validity, source timestamps, target association and freshness remain required. Bottle reporting requires a valid eight-frame batch, including at the acquisition budget limit; stale cached estimates cannot become a fresh successful report. Approximate observations remain labelled approximate even when they satisfy a loose preferred precision setting.

## Cleanup and verification

The bottle workflow hides the obsolete camera-angle, controller-reference, legacy-calibration and localization-method controls. The unused `vision_28_6.yaml` preset has been removed. Legacy modules remain available for other target classes and simulation fixtures. The independent static-data collector and its original images are unchanged.

Run `python -m pytest -q` in this release. Browser tests use `tests/browser/` and the existing read-only preview server. The added callback test executes production detector, filter, association and fusion method bodies with synthetic images/TF and mocked ROS transport/YOLO. The subsequent deployment also passed 97 focused tests under ROS Humble and `tests/integration_bottom_range.py` on isolated ROS domain 98. This transported synthetic far, normal, near-cropped and relaxed-alignment observations through the actual ROS nodes. The robot mission was then loaded in WAITING. This does not establish physical positioning accuracy; no movement test started.
