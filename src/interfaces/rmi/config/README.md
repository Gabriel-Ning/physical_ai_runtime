# RMI Embodiment Profile Templates (`src/interfaces/rmi/config/`)

This directory contains reference schema templates and specifications for configuring robot embodiments in the **ROS Manipulation Interface (RMI)**.

## Overview

RMI uses a unified YAML configuration schema that decouples robot-independent orchestration, action provider preemption, teleoperation, dataset recording, and sensing from specific robot hardware implementations.

## Directory Layout

* `templates/embodiment_profile.template.yaml` — Complete, annotated template defining all standard RMI configuration sections:
  1. `metadata`: Embodiment naming, type, vendor, and schema version.
  2. `host_roles`: RT host vs. Policy host component ownership.
  3. `groups`: Joint groupings, coordinate frames, and `ros2_control` controller interfaces.
  4. `compound_groups`: Composite groupings for multi-arm/manipulator views.
  5. `execution_manager`: Multi-agent preemption priorities, controller mappings, and source ingress endpoints.
  6. `agents`: Application dispatch loops and operating frequencies.
  7. `sensors`: Camera observations and sensor streaming QoS.
  8. `recorder`: MCAP dataset collection, staging, and zero-drop recording parameters.
  9. `teleoperators`: Master-slave teleoperation leader devices and source mappings.
  10. `features`: Middleware and safety guard feature flags.

## Authoring New Robot Profiles

When adding support for a new robot or embodiment:

1. Copy `templates/embodiment_profile.template.yaml` to `apps/profiles/<my_robot>.yaml`.
2. Fill in the joint names, controller manager namespaces, and ros2_control controller contracts matching your bringup stack.
3. Launch applications using your new profile:
   ```bash
   pixi run teleop --profile <my_robot>.yaml
   pixi run record --profile <my_robot>.yaml --task <task_name>
   pixi run replay --profile <my_robot>.yaml
   ```
