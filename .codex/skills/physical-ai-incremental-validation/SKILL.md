---
name: physical-ai-incremental-validation
description: Use for complex Physical AI robotics work that spans real hardware, sensors, calibration, motion planning, recording, replay, policy/data conversion, or deployment. Trigger when the user wants staged development, incremental validation, real-robot testing, ROS 2 launch/test work, hand-eye/camera/AprilTag integration, planner episode collection, or any task where Codex should split a risky robot workflow into safe, observable gates with explicit expected results.
---

# Physical AI Incremental Validation

## Core Contract

Treat complex robot AI work as a sequence of small validation gates. Each gate
must have:

- A narrow objective.
- A concrete command or launch entrypoint.
- Expected logs, topics, transforms, files, or visual observations.
- A pass/fail interpretation.
- A safe next step.

Prefer moving one layer at a time:

```text
static config/docs
  -> offline smoke check
  -> fake hardware / no-motion check
  -> real sensor perception check
  -> fake hardware motion rehearsal
  -> low-speed real hardware check
  -> recording/replay check
  -> larger integration
```

## Required Setup

Before changing ROS 2 workspace, build, launch, testing, realtime, simulation,
navigation, perception, hardware, or deployment behavior, also inspect the
matching project ROS 2 skill under:

```text
.codex/skills/ros2-engineering-skills/
```

Use this skill to structure the work; use the ROS 2 skills for domain-specific
implementation details.

## Workflow

1. Restate the current stage and the exact boundary.
   - Name what is already validated.
   - Name what is not being attempted yet.
   - Keep deferred work explicit.

2. Pick the smallest next closed loop.
   - For perception: verify image, detection, TF, and numeric pose before motion.
   - For motion: verify fake hardware before real hardware.
   - For data: verify topic/type/shape before full conversion.
   - For replay: verify fake hardware before real hardware.

3. Implement only what the next gate needs.
   - Avoid combining perception, planning, recording, and replay in one new step.
   - Add launch files or scripts only when they produce repeatable checks.
   - Keep motion generators conservative and transparent.

4. Give the user a deterministic test command.
   - Prefer `pixi run <task>` for project workflows.
   - Include key launch arguments when they matter.
   - State whether the command uses fake hardware or real hardware.

5. State expected evidence.
   - Good examples: specific log lines, topic names, TF frame names, file paths,
     dataset feature shapes, expected drop counts, RViz/Rerun/rqt observations.
   - Avoid vague outcomes like "it should work".

6. Interpret returned logs.
   - Identify the first failing gate.
   - Distinguish expected warnings from blockers.
   - If a process dies, inspect the process name and earliest error first.

7. Record learning.
   - Update stage docs/checklists with completed gates.
   - Record non-blocking issues as TODOs with symptom, impact, current status,
     and candidate fix.

## Safety Rules

- Do not send real hardware motion commands until perception, TF, and fake
  hardware motion gates have passed.
- Make launch defaults safe when possible: `teleop_mode:=none`, fake hardware,
  no CAN init, or no publisher for motion topics.
- When real hardware is required, say so explicitly and name the motion topic or
  controller being exercised.
- Prefer single-arm/no-gripper scope unless the current stage explicitly
  includes gripper, bimanual, or whole-body control.
- Keep GPU, IK, MPC, ML, and Python work outside realtime control loops.

## Communication Pattern

Use concise progress updates:

- "I am checking X because it is the next gate before Y."
- "This launch is safe because it does not publish motion commands."
- "Expected evidence is A, B, C."
- "This warning is not blocking because ..."
- "The next gate is ..."

When handing off to the user for real hardware, include:

- The exact command.
- Hardware assumptions.
- What they should observe.
- What log lines matter.
- What failure output to paste back.

## Example Gate Template

```text
Gate: Orbbec AprilTag pose in base_link
Command: pixi run stage5-track-b-marker-verify
Hardware: real Orbbec + real robot state, no motion commands
Expected:
  - rqt debug image draws the tag frame
  - RViz shows tag pose
  - log prints base_link -> tag36h11:4 and hover_xyz
Pass means:
  - camera, AprilTag, and static hand-eye TF are consistent enough for planning
Next:
  - fake-hardware marker-hover rehearsal
```

## Documentation Hygiene

When updating stage docs, keep the distinction clear:

- Completed gates: verified and reproducible.
- Current gate: one objective with one command.
- TODOs: known issue, impact, current status, candidate fix.
- Deferred: intentionally not part of the current stage.
