# Publish by Fleet Adapter (aka DispatchStatus)

# Fleet Adapter name
string fleet_name

# *optional and duplicated in TaskProfile
string task_id 

TaskProfile task_profile

uint32 state
uint32 STATE_QUEUED=0
uint32 STATE_ACTIVE=1
uint32 STATE_COMPLETED=2  # hooray
uint32 STATE_FAILED=3     # oh no
uint32 STATE_CANCELED=4
uint32 STATE_PENDING=5

# a brief summary of the current status of the task, for UI's
# *optional
string status

# submission_time is when the task was submitted to rmf_core
# *optional and duplicated in TaskProfile
builtin_interfaces/Time submission_time   

# when rmf_core actually began processing the task
builtin_interfaces/Time start_time

# When this message is a summary of an in-process task, the end_time field is
# an estimate. When this message is a summary of a completed or failed task,
# end_time is the actual time.
builtin_interfaces/Time end_time

# Allocated robot name
# *optional
string robot_name
