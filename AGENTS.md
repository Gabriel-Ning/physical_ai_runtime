# Project Agent Instructions

This repository keeps project-level agent skills in `.codex/skills`.

When working in this workspace:

- Treat `.codex/skills` as the canonical project skill directory.
- Before changing ROS 2 workspace, build, launch, testing, realtime, simulation, navigation, perception, hardware, or deployment behavior, inspect the matching `SKILL.md` under `.codex/skills/ros2-engineering-skills/`.
- Use `.codex/skills/karpathy-guidelines/SKILL.md` for general engineering style and code quality guidance when applicable.
- Prefer repository-local instructions over global defaults when they conflict.
- Do not duplicate skill content into editor-specific rule files; those files should point back to `.codex/skills`.
