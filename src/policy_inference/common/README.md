# Common policy runtime

Backend-neutral hot-path components shared by LeRobot and future GR00T/OpenPI
integrations:

- `PolicyIOContract` resolves action order and camera features once from the RMI
  embodiment profile.
- RMI `Robot`, `Agent`, and `Session` remain the only runtime observation and
  action abstractions. Backend packages bridge their native feature format to
  these objects without introducing another Robot or environment.

This package must not import Torch, LeRobot, or a model-specific SDK.
