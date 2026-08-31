# RMI Embodiment Profile Templates (`src/interfaces/rmi/config/`)

RMI application profiles live in `apps/profiles/`. They are the app API
(sensors, nodes, recorder, features), not the Execution Manager routing table.

Marvin points at the workstation EM capability config via
`execution_manager_config`; Piper and Franka currently embed the same
`groups` projection. No application profile declares provider routing.

## Layout

* `templates/embodiment_profile.template.yaml` — minimal application schema:
  1. `metadata`
  2. `host_roles` (RT vs workstation; workstation launch defaults)
  3. `groups` / controllers / `/execution/...` endpoints
  4. `compound_groups`
  5. `nodes` (`source_role` + resource/command contracts)
  6. optional sensors, recorder client defaults, teleoperators, and features

EM execution-capability template:

* `src/execution/execution_manager/config/templates/execution_manager_profile.template.yaml`

## Authoring

1. Copy `templates/embodiment_profile.template.yaml` → `apps/profiles/<my_robot>.yaml`.
2. Fill controller capabilities and application nodes. Do not add priorities or
   policy implementations to the EM config.
3. Point workstation / apps at that single file:

   ```bash
   pixi run teleop --profile <my_robot>.yaml
   ros2 launch piper_manipulation_workstation_launch piper_workstation.launch.py
   ```
