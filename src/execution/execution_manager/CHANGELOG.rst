^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package execution_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2026-09-05)
------------------
* Add startup-only ``use_sim_time`` support so MuJoCo / ``/clock`` can drive
  command-age checks, status publication, and trajectory-guard heartbeats.
* Keep the default ``use_sim_time:=false`` wall/steady path for real robots.
* Publish ``/execution_manager/authority_events`` with transient-local durability
  and enrich failed claim/release diagnostics.

0.2.0 (2026-08-31)
------------------
* Relay JTC and parallel-gripper Action feedback, results, errors, and cancel
  requests to the originating client.
* Serialize trusted external-source activation and release transitions outside
  ingress callbacks to prevent controller-manager response starvation.
* Reconcile released cached leases, restore only live default sources, and
  stabilize compound Action/source teardown behavior.
* Reject zero-stamped external streaming commands before source activation or
  downstream forwarding while preserving standard zero-stamped JTC goal
  semantics.
* Add Action lifecycle, concurrent ingress, stale-lease, liveness, and timestamp
  regression coverage.

0.1.0 (2026-08-24)
------------------
* Initial release of C++ Dynamic Authority Execution Manager daemon.
* Atomic multi-resource leasing, explicit preemption, typed envelope command routing.
* Downstream ros2_control controller switching and LeasedFollowJointTrajectory action proxying.
