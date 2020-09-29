# This msg is will be received by Fleet Adapter

string task_id # guid, or version (?)

# indicate task type which the robot will check if it is valid
TaskType type

# submission_time is when the task was submitted to rmf_core
builtin_interfaces/Time submission_time

# Duration which bidding is open
builtin_interfaces/Duration time_window

# Announce bidding task to all bidder
bool announce_all

# Optional: Specify bidders if announce_all is false
string[] fleet_names

# array of stopby place names
string[] itinerary
