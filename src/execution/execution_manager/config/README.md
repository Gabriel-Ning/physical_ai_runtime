# Execution Manager config

Execution capability files are workstation deployment inputs. Application
defaults remain in `apps/profiles/`.

This package keeps an **Execution Manager-scoped template** that documents the
YAML sections the node actually consumes:

* `config/templates/execution_manager_profile.template.yaml`
  — `metadata`, `max_command_age_s`, and `groups` only

For the full embodiment schema used by RMI apps (agents, sensors, recorder,
teleoperators, host_roles, …), see:

* `src/interfaces/rmi/config/templates/embodiment_profile.template.yaml`

The EM config never declares providers, priorities, or candidate policy. RMI
agents claim a resource/command contract dynamically at runtime.
