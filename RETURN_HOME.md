> Current release/deployment status: [README.md](README.md) and [DEPLOYMENT.md](DEPLOYMENT.md). Earlier dated status statements below are historical.

> Historical release note. Deployment IDs, state snapshots and validation counts below describe that release. Current source behavior and deployment status are in [README.md](README.md) and [SEARCH_RECOVERY.md](SEARCH_RECOVERY.md).

# Return to home

A fixed header button beside Stop requests return to the home recorded by the current task. It works in mapping and mission stages, including an idle failed/stopped attempt with a retained home. No arbitrary goal coordinates are accepted.

The owner first cancels any active or pending action and waits for its terminal result before dispatching home. Stop supersedes the pending return. Map, localization, navigation-lock, motion-permission and existing planner/gate checks remain in force. Arrival is verified using the existing home/stationary checks; ending a search early does not claim complete search coverage.

During deployment, the original recorded home was retained only on the same selected saved map. A stopped fresh mission may accept a current-position correction before any new motion starts; this does not overwrite the retained home.

Validation: initial full local suite 859 tests; follow-up 55 policy tests; 7 initial UI controls/layout checks and 3 final retained-home UI checks; 202 initial native focused tests and 8 final HTTP/ROS owner cancellation/return checks. Final native checks cover active return, late acceptance, Stop precedence, missing home, retained home and current-pose setup. No physical Return button click was made during deployment.
