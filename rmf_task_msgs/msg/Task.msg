# Unique ID to differentiate the task
string task_id

# ============================================================================
# Complusory fields during task reqeust (ADD method) and status update

# submission_time is when the task was first submited
builtin_interfaces/Time submission_time

# e.g. Delivery, Station.....
TaskType type

# array of stopby place/waypoints names
string[] itinerary

# ============================================================================
# Populated only by task action server after the task is queued

# when the task is first executed the task
# if the task is queued, this will be an estimate
builtin_interfaces/Time start_time

# When this message is a summary of an in-process task, the end_time field is
# an estimate. When this message is a summary of a completed or failed task,
# end_time is the actual time.
builtin_interfaces/Time end_time

# optional field
string robot_name # ???

# state
uint8 active_state
uint8 INVALID=0
uint8 QUEUED=1
uint8 EXECUTING=2
uint8 CANCELING=3

# todo: phase?

# a brief summary of the current status of the task, for UI's
string status
