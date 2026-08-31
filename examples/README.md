# RMI examples

These programs are workstation applications built on RMI. RMI supplies Robot
observations, action Node handles, sensors and recorder clients; planning remains in
the independent `src/motion_planning/motion_planners` workspace. The RT host
never imports RMI and never writes datasets.

## Validation status

Marvin real-hardware validation is complete. Examples 17--20 verified Quest 3
teleop, transparent preemption and recovery, episode recording, replay, and
long-duration trajectory recording. The current validation target is Franka
fake hardware with gamepad teleop and no camera nodes.

Demo 07 MCAP path uses the recorder started by `workstation_stack`. The `memory`
path is entirely in process. Camera demos additionally require their configured
camera publishers. Planner demos require cuRobo and a working CUDA environment.

Run commands from the repository root after `source install/setup.bash`.

## Catalog and reproduction commands

| Demo | Capability | Marvin command / note |
| --- | --- | --- |
| `01_context.py` | SDK topology, readiness, observations | `python examples/01_context.py --profile marvin_bimanual.yaml` |
| `02_policy_camera.py` | Policy plus camera observation | add camera publishers; use `--profile marvin_bimanual.yaml` |
| `03_teleop_preempt.py` | Policy/teleop arbitration and recovery | `python examples/03_teleop_preempt.py --profile marvin_bimanual.yaml --side left` |
| `04_plan_execute.py` | independent cuRobo planner to JTC | `python examples/04_plan_execute.py --profile marvin_bimanual.yaml --side left` |
| `05_policy_recovery.py` | policy interrupted by planner recovery | `python examples/05_policy_recovery.py --profile marvin_bimanual.yaml` |
| `06_partial_ownership.py` | concurrent disjoint left/right routes | `python examples/06_partial_ownership.py --profile marvin_bimanual.yaml` |
| `07_record_episode.py` | MCAP recorder client or memory replay buffer | `python examples/07_record_episode.py --profile marvin_bimanual.yaml --type memory --duration 3` |
| `08_ik_resolver.py` | independent cuRobo IK to joint stream | `python examples/08_ik_resolver.py --profile marvin_bimanual.yaml --side left --mode circle` |
| `09_joint_streamer.py` | VLA chunks or cuRobo MPC | `python examples/09_joint_streamer.py --profile marvin_bimanual.yaml --side left --backend vla_dummy` |
| `10_marker_teleop.py` | Cartesian/twist interactive marker | `python examples/10_marker_teleop.py --profile marvin_bimanual.yaml --side left` |
| `11_joint_jog.py` | interactive joint jog | `python examples/11_joint_jog.py --profile marvin_bimanual.yaml` |
| `12_gripper_control.py` | gripper command over a declared side route | `python examples/12_gripper_control.py --profile marvin_bimanual.yaml --side left --width 0.03` |
| `13_replay_episode.py` | MCAP actions replayed through a Replay Node | requires an existing episode; see `--help` |
| `14_piper_leader_teleop.py` | Piper leader/follower teleop | requires Piper leader topics and profile |
| `15_test_preempt_statemachine.py` | deterministic simulated preemption harness | see `--help` |
| `16_franka_gamepad_teleop.py` | Unified Action Node JSPC policy with EM gamepad takeover and MCAP recording | `python examples/16_franka_gamepad_teleop.py --profile fr3_pika_single_arm.yaml` after the setup below |
| `17_marvin_quest3_teleop.py` | Unified Action Node bimanual policy with EM Quest 3 teleop takeover and MCAP recording | see setup below |
| `18_marvin_joint_slider.py` | GUI sliders for both Marvin arms and Pika grippers through EM | `python examples/18_marvin_joint_slider.py --profile marvin_bimanual.yaml` |
| `19_marvin_homing_replay.py` | Smooth homing followed by MEMORY-source episode replay | `python examples/19_marvin_homing_replay.py --profile site/marvin_bimanual_no_cam.yaml` |
| `20_marvin_trajectory_recording.py` | Smooth homing followed by recording a two-cycle bimanual JTC trajectory | `python examples/20_marvin_trajectory_recording.py --profile site/marvin_bimanual_no_cam.yaml` |

## Franka fake-hardware gamepad setup

This is the current validation gate. Start each layer in a separate terminal
after building and sourcing the workspace. It uses fake FR3 and fake Pika
hardware; it does not connect to FCI, the gripper serial port, or cameras.

1. Start the RT side with the existing stack. Load both the fake FR3 arm and
   fake Pika gripper, and disable cameras:

   ```bash
   ros2 launch franka_manipulation_rt_launch rt_stack.launch.py \
     use_fake_hardware:=true load_pika_hardware:=true \
     with_cameras:=false use_rviz:=true cpu_affinity:=none
   ```

2. Start the workstation side. Its normal launch owns gamepad teleop,
   Execution Manager, and episode recorder:

   ```bash
   ros2 launch franka_manipulation_workstation_launch workstation_stack.launch.py
   ```

3. Run the Unified Action Node example:

   ```bash
   python examples/16_franka_gamepad_teleop.py
   ```

4. Verify the expected graph before starting the recording:

   ```bash
   ros2 control list_controllers
   ros2 topic hz /joint_states --window 20
   ros2 topic echo /execution_manager/authority_status --once
   ros2 node list | grep -E 'realsense|d405|fisheye' || echo 'PASS: no camera nodes'
   ```

Follow the prompt to start recording. The in-process `DummyPolicy` node streams
candidate joint actions after an 8 s JTC move to the profile-configured homing
pose. Hold **L1/LB** on the gamepad to trigger transparent
teleoperation preemption by Execution Manager; release it to resume policy execution.
Press Enter to finalize and save the synchronized MCAP dataset episode.

The `fr3_pika_single_arm.yaml` profile selects
`apps/recording/franka_manipulation_no_cam.yaml`. Camera streams remain known
to the recording schema but are optional and do not participate in the start
gate.

## Marvin reproduction reference (completed)

1. Start the RT side for Marvin humanoid (dual-arm fake hardware):

   ```bash
   ros2 launch marvin_manipulation_rt_launch rt_stack.launch.py \
     use_fake_hardware:=true with_cameras:=false cpu_affinity:=none
   ```

2. Start the workstation side (Execution Manager and Episode Recorder):

   ```bash
   ros2 launch marvin_manipulation_workstation_launch workstation_stack.launch.py \
     with_cameras:=false with_teleop:=true
   ```

3. Run Example 17 with the no-camera profile:

   ```bash
   python examples/17_marvin_quest3_teleop.py \
     --profile site/marvin_bimanual_no_cam.yaml
   ```

Follow the prompt to start recording. The in-process `DummyPolicy` generates
coordinated bimanual JSPC motions across all 16 joints (both 7-DOF arms and dual grippers).
Squeeze both Quest 3 controller grips to trigger transparent teleoperation takeover;
release the squeeze grips to seamlessly resume autonomous policy motion.
Press Enter to finalize and save the synchronized MCAP dataset episode.

## Fast validation

```bash
python -m compileall -q examples
for f in examples/[0-9][0-9]_*.py; do python "$f" --help >/dev/null; done
```

`--help` validates imports and CLI wiring but does not prove ROS graph, controller,
CUDA, camera, recorder, or physical-leader readiness. Validate those dependencies
at the gate documented for each demo before allowing motion.
