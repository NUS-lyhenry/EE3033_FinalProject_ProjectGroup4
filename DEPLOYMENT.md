# Automatic mapping position-only arrival

The physical release `mapping_position_only_20261105_719b66866f` was activated on 2026-11-05. The console is http://127.0.0.1:8765/#/workflow. The predecessor is `persistent_settings_20261105_dfc48c197f`.

The automatic mapping behavior tree selects a dedicated 0.12 m position-only checker. The supervisor still verifies the fresh matching route and the actual XY endpoint. It observes path heading while traveling and skips the final heading watchdog at position-only arrival. The RPP wrapper returns zero on the arrival tick for position-only checkers, including the existing home checker. Manual waypoints and target observation retain their assigned headings. Home verification continues to require its original position and stationary hold.

1130 local tests passed and one skipped. Both ROS packages built in 38.7 seconds. Seven isolated supervisor tests passed. The installed Nav2 controller accepted a mapping goal at 8 cm with 2 rad yaw error and published zero raw commands. The assigned-heading goal at the same position requested a turn.

Before switching, physical odometry passed the existing stationary check. The deployment preserved all 48 saved map files and seven saved settings files. The new stage restored the saved values and initially reported WAITING with zero output. The installed behavior tree, controller library, and five effective checker configurations were verified. The operator then began mapping and the console showed two goals reached. The agent sent no Start or physical goal. Logs and receipts are in `validation/20261105/`.
