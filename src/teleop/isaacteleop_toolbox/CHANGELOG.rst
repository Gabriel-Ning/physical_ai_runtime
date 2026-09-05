^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package isaacteleop_toolbox
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-07-11)
------------------
* Add the Quest bimanual clutch-relative teleoperation reference application.
* Isolate workspace-local CloudXR host-client compatibility and offline assets.
* Add installed CloudXR setup and source-only live/replay launch entrypoints.
* Publish stamped pose targets, clutch snapshots, and versioned status metadata.
* Document IsaacTeleop as an environment-owned dependency (not rosdep/setuptools).
* Split detailed docs under ``docs/``; keep the homepage README minimal.
* Validate source-only operation in the originating Physical AI Runtime
  workspace; robot-app fake/real hardware gates remain outside this package.
* Contributors: Gabriel-Ning
