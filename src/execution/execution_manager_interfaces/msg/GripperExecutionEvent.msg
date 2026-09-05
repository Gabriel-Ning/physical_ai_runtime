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
sensor_msgs/JointState command
sensor_msgs/JointState result_state
bool stalled
bool reached_goal
