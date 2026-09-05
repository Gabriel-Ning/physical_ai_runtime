# Immutable source output. Identity is (source_id, bundle_id, candidate_id).
uint8 FULL_TRAJECTORY=1
uint8 CHUNK=2

std_msgs/Header header
string source_id
string part
string bundle_id
builtin_interfaces/Time generated_at
builtin_interfaces/Time start_state_stamp
sensor_msgs/JointState start_state
uint8 execution_mode
Candidate[] candidates

# Reserved correlation fields for future replan/re-inference requests.
string request_id
string parent_bundle_id
