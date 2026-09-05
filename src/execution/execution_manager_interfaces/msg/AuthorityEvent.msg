uint8 CLAIMED=1
uint8 PREEMPTED=2
uint8 RELEASED=3
uint8 TRANSITION_FAILED=4

std_msgs/Header header
string event_id
uint8 type
string lease_id
string related_lease_id
uint8 source_role
string source_instance
string[] resources
string reason
diagnostic_msgs/KeyValue[] metadata
