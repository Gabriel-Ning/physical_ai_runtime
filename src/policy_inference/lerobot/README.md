# LeRobot policy node

The LeRobot integration implements the same producer contract as an in-process
example policy:

```text
Robot.get_observation()
  -> LeRobotPolicyNode.select_action()
  -> RMI Node.submit()
  -> Execution Manager
```

The RMI profile supplies ordered state/action features, camera sources and the
named `Policy` Node binding. The LeRobot runtime owns inference and buffering;
it does not claim authority, switch controllers or implement teleop scheduling.

The executable creates cameras through `Context`, so their latest samples are
present in the same Robot observation passed to `select_action()`. Returning
`None` during model warm-up is a valid no-candidate cycle.
