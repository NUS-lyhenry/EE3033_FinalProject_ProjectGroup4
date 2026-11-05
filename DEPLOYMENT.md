# Manual clearance and measured body sweep

The physical release `manual_clearance_20261105_89a6af1ef1` was activated on 2026-11-05. The measured-profile collision protection radius now accepts 0.07–0.30 m and retained the saved 0.116801 m value. Small settings supplement the requested circle with a swept, padded wheel-inclusive rectangle. An isolated ROS node on the robot accepted 0.07 m, rejected 0.069 m and verified side-wheel protection. No physical motion was started.

The three presets and narrow-gap switch were removed from the managed console. The existing 0.17 m planning radius, 0.18 m inflation radius and 0.11 m exploration clearance were migrated to explicit manual settings. Effective clearances and other settings matched their pre-switch readback. Both ROS packages built, served UI assets matched, all 68 map files were preserved, and the mapping stage was ready in `WAITING` with zero output.

# Previous release: collision protection radius minimum

The physical release `collision_radius_minimum_20261105_bf7acb9a40` was activated on 2026-11-05. The console reported a swept collision radius range of 0.112801–0.30 m and preserved the current value and default of 0.116801 m. Both ROS packages built and served UI files matched the release. The mapping stage was ready in `WAITING` with zero output. All 68 map files and seven operator settings files passed checksum verification. The agent sent no Start or navigation goal.

# Previous release: turning stop distance minimum

The physical release `turn_stop_minimum_20261105_b6c3dc5aed` was activated on 2026-11-05. The console read back a turning stop distance minimum of 0.070081 m, the preserved operator value of 0.116801 m, and the collision radius floor of 0.116801 m. Both ROS packages built and the served UI files matched the release. The mapping stage was ready in `WAITING` with zero velocity. All 68 map files and seven operator settings files passed checksum verification. The agent sent no Start or navigation goal.

# Previous release: navigation, clearance and grid settings

The physical release `navigation_grid_settings_20261105_e275b2204b` was activated on 2026-11-05. The live settings page is http://127.0.0.1:8765/#/navigation. Both ROS packages built on the robot. The loaded controller library and the served HTML, JavaScript and CSS matched the release. The mapping stage was ready in `WAITING` with zero final velocity.

The saved navigation profile was carried forward, including the 7 cm ordinary lookahead. The new corner lookahead read back at its 8 cm default. SLAM resolution read back at 5 cm and the navigation grid at approximately 1.67 cm. All 68 map files and seven settings files passed checksum verification. The previous release remains available for rollback. The agent sent no Start or navigation goal.

Disk cleanup removed an inactive downloaded VS Code Remote runtime and reclaimed approximately 643 MiB. Available space after deployment was approximately 863 MiB.

# Previous release: automatic mapping position-only arrival

The physical release `mapping_position_only_20261105_719b66866f` was activated on 2026-11-05. The console is http://127.0.0.1:8765/#/workflow. The predecessor is `persistent_settings_20261105_dfc48c197f`.

The automatic mapping behavior tree selects a dedicated 0.12 m position-only checker. The supervisor still verifies the fresh matching route and the actual XY endpoint. It observes path heading while traveling and skips the final heading watchdog at position-only arrival. The RPP wrapper returns zero on the arrival tick for position-only checkers, including the existing home checker. Manual waypoints and target observation retain their assigned headings. Home verification continues to require its original position and stationary hold.

1130 local tests passed and one skipped. Both ROS packages built in 38.7 seconds. Seven isolated supervisor tests passed. The installed Nav2 controller accepted a mapping goal at 8 cm with 2 rad yaw error and published zero raw commands. The assigned-heading goal at the same position requested a turn.

Before switching, physical odometry passed the existing stationary check. The deployment preserved all 48 saved map files and seven saved settings files. The new stage restored the saved values and initially reported WAITING with zero output. The installed behavior tree, controller library, and five effective checker configurations were verified. The operator then began mapping and the console showed two goals reached. The agent sent no Start or physical goal. Logs and receipts are in `validation/20261105/`.
