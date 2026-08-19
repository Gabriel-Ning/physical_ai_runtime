# Physical AI Runtime Applications Suite (`apps/`)

Production-ready, standalone CLI applications built on top of the **RMI Python SDK**.

All applications can be invoked directly with `pixi run <app>` or `python apps/<app>.py`.

---

## 1. `pixi run teleop` — Interactive Robot Teleoperation

Connects Piper Master-Slave leader arms or input devices for real-time remote control:

```bash
# Piper Bimanual Master-Slave Teleoperation:
pixi run teleop --profile piper_bimanual.yaml --side both

# Franka / Single-Arm Teleoperation:
pixi run teleop --profile fr3_pika_single_arm.yaml --device keyboard
```

---

## 2. `pixi run record` — Multi-Modal Episode Dataset Recorder

Synchronously records multi-modal robot demonstrations (Cameras + Joint States + Actions) into immutable `.mcap` datasets with SHA-256 integrity seals:

```bash
# Record 1 episode (30 seconds @ 30Hz):
pixi run record --profile piper_bimanual.yaml --task bimanual_pickup --duration 30

# Record 5 consecutive episodes with countdown:
pixi run record --profile piper_bimanual.yaml --task bimanual_pickup --episodes 5 --duration 20
```

---

## 3. `pixi run replay` — 1:1 Native Trajectory Replayer

Performs strict 1:1 native timestamp-paced trajectory replay directly from recorded MCAP datasets:

```bash
# Replay latest recorded episode at 1:1 native rate:
pixi run replay --profile piper_bimanual.yaml

# Replay specific MCAP episode:
pixi run replay --profile piper_bimanual.yaml \
    --episode data/episodes/piper_policy/episode_000000/episode_000000.partial_0.mcap
```
