uint8 UNOWNED=0
uint8 TRANSITIONING=1
uint8 OWNED=2
uint8 FAULT=3

string resource
uint8 authority_state
string lease_id
uint8 source_role
string source_instance
string command_contract
string requested_controller
string[] observed_controllers
