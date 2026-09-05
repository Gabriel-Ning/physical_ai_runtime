"""RMI-backed leaf nodes.

Every node here calls the RMI SDK synchronously (``RobotFacade.control()``/
``.execute()``, ``RecorderFacade.episode()``); none of them talk to EM,
ros2_control, or ROS transports directly, and none of them ``await em.*``.
"""
