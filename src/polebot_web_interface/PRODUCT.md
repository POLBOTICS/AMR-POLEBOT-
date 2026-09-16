# POLEBOT Navigation

POLEBOT Navigation is the browser workspace for operating and inspecting this robot's ROS 2 navigation. Its primary user is an operator or developer who needs to see the map, robot, laser scan, path, costmaps, and Nav2 readiness together before issuing a destination.

The intended workflow follows the familiar RViz/Foxglove pattern: inspect the scene and display layers, select a goal position, hold and drag to set yaw, inspect the preview, then explicitly press **Send**. Releasing the pointer must not send a navigation goal. Initial-pose placement uses the same position-and-heading interaction. Keep numeric heading adjustment available.

## Durable preferences

- Keep the existing 3D viewer orientation and robot/LiDAR calibration. The accepted scene is the reference for alignment.
- Use a dense, dark operating workspace with display controls, a central scene, and navigation actions/status visible together.
- Keep a separate Send button after preview, as explicitly requested by the user.
- Present costmap data using the RViz cost palette, with independent opacity controls and a minimum visible cost control. The minimum control changes visualization only; it must not tune Nav2 inflation radius or cost scaling.
- Use explicit readiness, action progress, cancellation, and uncertain/disconnected states; do not infer success from sending a request.

## Scope and boundaries

The implemented Navigation workspace supports display toggles, map framing/follow controls, goal and initial-pose placement, lifecycle readiness, Nav2 action feedback/cancellation, and global/local costmap display including incremental updates. It is inspired by the [Foxglove 3D panel](https://docs.foxglove.dev/docs/visualization/panels/3d), not a claim of full Foxglove feature parity.

The calibrated LiDAR mapping `(ly + 0.496, -lx)`, STL scale `0.001` with zero mesh rotation, pose smoothing `0.4`, and existing map texture flip remain implementation invariants. Costmap frame transforms apply to costmaps, not a rewrite of the robot or laser transforms. Navigation requests use rosbridge `send_action_goal`/`cancel_action_goal` with the active action ID; lifecycle readiness comes from `get_state` responses.

## Validation boundary

The frontend test suite and synthetic desktop/mobile browser fixtures exercise behavior and layout without robot hardware. Actual robot/Nav2 integration validation is still required, including the deployed rosbridge action protocol, lifecycle services, TF, topic frames, costmap updates, goal execution, and cancellation. Development UI fixtures are not shipped as application features.

## Reference provenance

The supplied costmap visual reference is `/home/mirae/Downloads/applsci-12-08084-g006.jpg`. The palette implementation reference is [RViz palette_builder.cpp](https://github.com/ros2/rviz/blob/rolling/rviz_default_plugins/src/rviz_default_plugins/displays/map/palette_builder.cpp): costs 1–98 ramp from blue toward red, 99 is cyan, and 100 is magenta. These are occupancy-grid cost values, not physical inflation widths.
