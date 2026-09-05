uint8 ACCEPTED=1
uint8 SUCCEEDED=2
uint8 CANCELED=3
uint8 ABORTED=4

std_msgs/Header header
string execution_id
string lease_id
uint8 source_role
string source_instance
string resource
uint8 state
trajectory_msgs/JointTrajectory trajectory
int32 error_code
string error_string
