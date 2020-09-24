# This msg is will be received by Fleet Adapter

string task_id
uint32 request_version # random generated number

# Selected Fleet Adapter / Target server
string fleet_name

# action
uint8 action
uint8 CANCEL_TASK=0
uint8 QUEUE_TASK=1

# submission_time is when the request was submitted to rmf_core
builtin_interfaces/Time submission_time

# task type
TaskType type

# array of stopby place names
string[] itinerary

# Behavior # ??
