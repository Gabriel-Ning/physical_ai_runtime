cmake_minimum_required(VERSION 3.16)
project(execution_manager_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Candidate.msg"
  "msg/CandidateBundle.msg"
  "msg/CommandEndpoint.msg"
  "msg/ResourceClaim.msg"
  "msg/ResourceAuthority.msg"
  "msg/SourceDescriptor.msg"
  "msg/SourceRegistry.msg"
  "msg/AuthorityStatus.msg"
  "msg/AuthorityEvent.msg"
  "msg/GripperExecutionEvent.msg"
  "msg/GripperExecutionFeedback.msg"
  "msg/LeasedJointReference.msg"
  "msg/LeasedPoseReference.msg"
  "msg/LeasedTwistReference.msg"
  "msg/TrajectoryExecutionEvent.msg"
  "msg/TrajectoryExecutionFeedback.msg"
  "srv/ClaimControl.srv"
  "srv/ReleaseControl.srv"
  "action/LeasedFollowJointTrajectory.action"
  "action/LeasedParallelGripperCommand.action"
  DEPENDENCIES
    builtin_interfaces
    control_msgs
    diagnostic_msgs
    geometry_msgs
    moveit_msgs
    sensor_msgs
    std_msgs
    trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
