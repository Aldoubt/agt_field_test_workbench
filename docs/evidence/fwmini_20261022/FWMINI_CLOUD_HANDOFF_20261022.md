# FW-mini MID360 + editable RViz interaction handoff | 2026-10-22

Dedicated branch: Aldoubt/fwmini_navigation_lab feature/fwmini-mid360-route-workbench-20261022
V3 companion: Aldoubt/agt_navigation_v3 feature/fwmini-field-navigation-20261022
FW-mini base: local functional baseline 9910b4908b7528092d16c3ecd6534e8ed3066936 including earlier local FW-mini commits. Neither branch is merged into main.

## FW-mini native sources
src/fwmini_bringup: shared field chassis geometry and launch.
src/fwmini_description: MID360 mounting transform and Xacro.
src/fwmini_chassis_adapter: CtrlCmd vendor ABI isolation, watchdog/velocity bridge.
src/fwmini_multi_nav_bringup: Nav2, FAST-LIO2, Livox clock, global relocalization integration, S2 UI launch.
src/agt_fastlio_adapter: LIO pose/frame/velocity adapter and tests.
src/agt_navigation_interfaces: ROS navigation interfaces used by the field branch.
tools/field_20261022: field-tested build, map preparation, nav, UI and read-only validation commands. They were originally installed in ~/ros2_ws/scripts.
docs/field_20261022: field notes and verification boundaries.

## Full editable interaction code snapshot
integrations/v3_fwmini_ui/packages contains:
agt_rviz_patrol, agt_rviz_route_tools, agt_navigation_capability,
agt_navigation_supervisor, agt_robot_interfaces, agt_operator_console.
The source is a copy of the V3 field-branch versions rather than just launch wrappers.
It is NOT a git submodule. Cloud changes to the snapshot are not automatically applied to the V3 repo. Synchronize explicitly before mainline integration.

WARNING: When both repositories are checked out, never let colcon discover duplicate ROS package names. Use explicit --base-paths to select only one copy. The separate UI overlay intentionally lives outside the default src path. ROS 2 Humble and V3 localization/Nav2/mapping dependencies are still required. This is not a self-contained system image.

## Mode and tests
FW-mini uses start_connection_policy=drawn_only, max distance 0.35m,
max lateral 0.12m, max heading 25 degrees and backward tolerance 0.05m.
DWB lateral vy is still disabled. No CAN motion enable changes were made.
RouteWorkbench 45 pytest PASS, Supervisor 8 pytest PASS, isolated FollowRoute
and ABI tests passed, but real final path-tracking precision NOT VERIFIED.
No PCD maps or rosbag files are included.

## Dependency warning: two incompatible yhs_can_interfaces installations
FW-mini vendor CtrlCmd requires FOUR fields (gear, x_linear, z_angular, y_linear).
Old ROS2_WS CtrlCmd had THREE fields (gear, linear, angular). Both publish the same type name; wrong ABI causes a valid-looking ROS graph but no meaningful vehicle motion.
Cloud build must obtain the licensed FW-mini vendor yhs_can_interfaces package separately (do not copy proprietary vendor drops into this repo) and compile it in the SAME install overlay as fwmini_chassis_adapter, so its installed library can be selected by agt_guarded_adapter.launch.py.
Reproduced build on Ubuntu 22.04/Humble with explicit colcon --base-paths including external vendor/yhs_can_interfaces and four repository packages:
yhs_can_interfaces, agt_navigation_interfaces, agt_fastlio_adapter, fwmini_chassis_adapter, fwmini_multi_nav_bringup -> 5 packages PASS.
Building against ros2_ws/install/yhs_can_interfaces (old message type) fails at compile time with no ctrl_cmd_x_linear.
See src/fwmini_chassis_adapter/launch/agt_guarded_adapter.launch.py for runtime library selection/contract.
