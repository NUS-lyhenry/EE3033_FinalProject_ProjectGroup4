> Current release/deployment status: [README.md](README.md) and [DEPLOYMENT.md](DEPLOYMENT.md). Earlier dated status statements below are historical.

> Historical release note. Deployment IDs, state snapshots and validation counts below describe that release. Current source behavior and deployment status are in [README.md](README.md) and [SEARCH_RECOVERY.md](SEARCH_RECOVERY.md).

# Recovery reverse switch

Deployed release: `reverse_toggle_20261101_b64b3e2b58`.

Advanced → Navigation recovery → Recovery reverse. Default ON. OFF replaces the second recovery backup with a bounded refresh/replan retry; retry count and absolute deadline remain unchanged. No changes to footprint, unknown-space policy, lidar cutoff or reverse geometry.

Saved in the existing session `recovery.json`, propagated through mapping and mission launch, and live readback verified. Edits require an idle owner, a stopped command and an idle supervisor. Failed writes roll back; an unverified rollback blocks new driving until reapplied.

Validation: 849 local tests; 147 native focused tests; 10 native action-supervisor tests including backup count zero when OFF and default backup retained when ON; 4 HTTP/ROS/persistence tests; 2 UI tests. The live UI reported ON with the setting locked during the user's running mission. No physical movement test was started by the deployment.
