# This msg is will be received by Fleet Adapter

string task_id # guid, or version (?)
TaskType type

# submission_time is when the task was submitted to rmf_core
builtin_interfaces/Time submission_time

# time which bidding will close
builtin_interfaces/Time deadline

# Potential Bidder
string[] fleet_names

# array of stopby place names
string[] itinerary
