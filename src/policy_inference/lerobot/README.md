# LeRobot policy runtime

This package uses LeRobot's native abstractions instead of implementing another
policy registry or action queue:

- `load_policy_bundle()` loads the checkpoint's `PreTrainedPolicy` and serialized
  pre/post `PolicyProcessorPipeline`s.
- `RmiToLeRobotObservationBridge` projects one native `rmi.Observation` into
  the raw values consumed by LeRobot's `build_dataset_frame()`.
- `LeRobotToRmiActionBridge` converts one postprocessed LeRobot action into
  native `rmi.Action` objects. `rmi.Robot` remains the only Robot abstraction,
  and every command still flows through its active `Session`.
- LeRobot `SyncInferenceEngine` is the compatibility baseline. LeRobot
  `RTCInferenceEngine` owns asynchronous chunk inference and `ActionQueue` for
  policies that implement RTC semantics.
- Profile feature keys are the canonical dataset/policy names. Their `source`
  fields remain free to point at embodiment-specific camera names and ROS topics.
- `validate_policy_compatibility()` checks checkpoint type and feature shapes.
  New datasets/checkpoints should also carry `policy_contract.json`, because a
  flat vector shape alone cannot prove joint element order.
- `synthetic_sync_dry_run()` performs one native LeRobot inference without ROS,
  an RMI session, or any motion output.
- Dataset converters should call `make_dataset_features()` and
  `write_contract_manifest()` so training and deployment share exactly the same
  camera keys and ordered joint components.

The ROS node keeps subscription callbacks short, runs LeRobot inference outside
executor callbacks, and acquires one RMI `Session` per active rollout. The hot
path is deliberately explicit:

```text
Session.observe()
  -> RMI-to-LeRobot observation bridge
  -> native LeRobot frame / processors / policy / action queue
  -> LeRobot-to-RMI action bridge
  -> Robot.send_action(..., observation=same_snapshot)
```

Initial validation must use `publish_to_robot:=false` or fake hardware.

The synchronous validation node now has a safe no-motion default:

```bash
python apps/policy.py \
  --profile apps/profiles/piper_bimanual.yaml \
  --checkpoint /path/to/pretrained_model \
  --task "pick up the object" \
  --device cuda
```

It only acquires an RMI lease and publishes actions when
`--publish-to-robot` is explicitly supplied. Native RTC node execution remains
the next gate after synchronous checkpoint validation.

## Deferred real-robot validation gates

The v0 offline integration is accepted without the following two items. Keep
them as explicit gates for the first real-robot rollout, where the complete
startup and failure-recovery path can be exercised safely:

1. **Warm up inference before acquiring control authority.** Wait for a valid
   observation, run one full bridge/processor/policy inference without an RMI
   `Session`, reset the inference engine and its action queue, and only then
   acquire the `Session`. On the validation GPU, cold inference took roughly
   286--421 ms while the first inference after reset took about 15 ms. This
   prevents compilation and allocation latency from entering the first command
   chunk sent to the robot.
2. **Carry the feature contract from training into the checkpoint.** The dataset
   converter generates `policy_contract.json`; the training/export workflow
   should validate it and copy that exact contract into each `pretrained_model`
   directory. Runtime must validate the checkpoint-owned contract rather than
   regenerate one from the deployment profile. The current legacy validation
   checkpoint has no contract: its profile, converted dataset, and statistics
   are consistent with the expected joint order, but they do not provide strict
   checkpoint provenance.

The real-robot gate must explicitly verify the ordered action components:
`left_joint1..6`, `left_gripper_joint1`, `right_joint1..6`,
`right_gripper_joint1`. Do not begin motion if the dataset, checkpoint contract,
and runtime profile disagree.

## Offline validation

```bash
PYTHONPATH=src PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .pixi/envs/lerobot/bin/python -m pytest -q src/policy_inference/tests
```
