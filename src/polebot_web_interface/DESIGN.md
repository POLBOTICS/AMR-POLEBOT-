---
name: POLEBOT Navigation
description: Dense ROS 2 navigation workspace centered on the calibrated 3D scene.
colors:
  primary: "#91b8ff"
  action: "#42659b"
  action-hover: "#527bb7"
  selected: "#304366"
  canvas: "#17191d"
  surface: "#22252b"
  pane-heading: "#282b32"
  border: "#383c45"
  text-primary: "#eceef2"
  text-secondary: "#b0b6c2"
  text-dim: "#a0a7b4"
  ready: "#3fb950"
  danger: "#f85149"
  warning: "#d29922"
  focus: "#a7caff"
typography:
  body:
    fontFamily: "Inter, sans-serif"
    fontSize: "13px"
  title:
    fontFamily: "Inter, sans-serif"
    fontSize: "16px"
    fontWeight: 600
    letterSpacing: "0.02em"
  label:
    fontFamily: "Inter, sans-serif"
    fontSize: "12px"
    fontWeight: 600
  telemetry:
    fontFamily: "JetBrains Mono, monospace"
    fontSize: "13px"
    fontWeight: 600
rounded:
  field: "3px"
  control: "4px"
  inset: "6px"
spacing:
  compact: "8px"
  control: "10px"
  section: "14px"
  roomy: "16px"
components:
  button-primary:
    backgroundColor: "{colors.action}"
    textColor: "white"
    rounded: "{rounded.control}"
    padding: "10px"
    width: "100%"
  button-primary-hover:
    backgroundColor: "{colors.action-hover}"
  heading-field:
    backgroundColor: "{colors.canvas}"
    textColor: "{colors.text-primary}"
    rounded: "{rounded.field}"
    padding: "5px 7px"
    width: "78px"
---
# Design System: POLEBOT Navigation

## Overview

**Creative North Star: "The Navigation Workbench"**

A dense, dark operating workspace organized around the calibrated 3D scene. Quiet charcoal surfaces and restrained blue controls keep the robot, map, scan, and costmap signals legible. This description records the implemented RViz/Foxglove-inspired direction; it does not introduce a new brand identity.

**Key Characteristics:**
- Dense operating controls around a dominant scene.
- Flat, separated panes with independently scrollable controls.
- Explicit visual feedback before and after navigation commands.

## Colors

The primary accent is a soft instrument blue; neutral charcoal surfaces establish the hierarchy. Frontmatter values record the effective Navigation workspace styles, including later CSS overrides.

Primary blue marks selected tools and pose values. The deeper action blue fills Send; its lighter variant marks hover. Ready green, danger red, and warning amber communicate operational states alongside text. Primary, secondary, and dim text form a restrained hierarchy on the canvas, surface, and pane-heading backgrounds.

**The Data Palette Rule.** Scene colors encode data and must not be replaced by the control accent. The RViz cost ramp covers values 1–98 from blue toward red, with cyan at 99 and magenta at 100. Global and local costmap opacity default to 0.30 and 0.65. Their sliders affect rendering only. The visible-cost threshold defaults to 1 and must not change real inflation widths.

The source palette is documented in [RViz palette_builder.cpp](https://github.com/ros2/rviz/blob/rolling/rviz_default_plugins/src/rviz_default_plugins/displays/map/palette_builder.cpp); the supplied visual reference is recorded in PRODUCT.md.

## Typography

Inter carries controls and explanatory copy; JetBrains Mono carries coordinates, topic/node identifiers, and telemetry. Use the existing 10–13px utility hierarchy rather than promotional display type. Navigation state uses an 18px bold mono readout; explanatory copy keeps generous line-height within the compact panes. Tabular numerals stabilize pose values. Topic labels may truncate visually while retaining a full title/accessible description.

## Layout

At desktop widths, the Displays pane is 245px and the action pane 302px, with the remaining width assigned to the central scene. At 1150px and below these reduce to 205px and 265px. Pane sections use 14–16px padding and small gaps; separators carry structure without adding nested cards.

At 850px and below, the page scrolls and stacks the scene, navigation actions, then Displays. Navigation initially uses two columns; at 450px and below both controls and Displays become single-column. The scene keeps a usable minimum height and the toolbar wraps. Header navigation remains available above the workspace.

## Elevation & Depth

The Navigation workspace is primarily flat: tonal surfaces, fine borders, and pane headings distinguish regions. Primary buttons have no shadow or hover lift. The inherited floating teleoperation overlay uses a dark shadow and blur to separate it from the scene. Reduced-motion preferences disable transitions and animations; the connection badge itself stays static.

## Shapes

Small control corners and compact bordered fields suit the operating environment. The effective toolbar and primary action radius is the control token; heading inputs use the field token. Status badges retain rounded capsule shapes to distinguish state from editable fields.

## Components

The toolbar uses outlined controls with blue border/text on hover and a tinted fill for the active tool. Send uses the filled primary action; cancellation uses a red border and tint. Disabled actions are visibly subdued and do not lift on hover. Inputs and buttons receive a two-pixel focus outline with three-pixel offset.

Display rows combine a checkbox, colored swatch, name, and topic. Costmap controls add opacity, update metadata, a cost legend, and a minimum visible cost slider. Goal preview uses coordinates and heading adjustment next to the separate Send action. Node readiness and navigation feedback use text as well as color. The scene footer holds coordinates and current interaction guidance.

**The Preview Rule.** Click-hold-drag selects position and yaw. Pointer release completes the preview; only the explicit Send action dispatches a navigation goal.

The [Foxglove 3D panel](https://docs.foxglove.dev/docs/visualization/panels/3d) is an interaction/layout reference, not a feature-completeness target.

## Do's and Don'ts

- Do keep scene data visually distinct from control-state colors.
- Do retain visible focus and textual status alongside status colors.
- Do retain the separate Send action after goal preview.
- Do show navigation actions before display settings when the workspace stacks.
- Don't alter the established viewer orientation or robot/LiDAR calibration for visual polish.
- Don't describe the minimum visible cost slider as a Nav2 inflation setting.
- Don't use synthetic fixture data as evidence of a working robot connection.
