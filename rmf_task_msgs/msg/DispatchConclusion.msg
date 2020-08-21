# This msg is will be received by Fleet Adapter

string task_id

# submission_time is when the task was submitted to rmf_core
builtin_interfaces/Time submission_time

# Selected Fleet
string fleet_name

# Selected robot
string robot_name

# array of stopby place names
string[] itinerary
