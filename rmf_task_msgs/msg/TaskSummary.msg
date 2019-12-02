string task_id

uint32 state
uint32 STATE_QUEUED=0
uint32 STATE_ACTIVE=1
uint32 STATE_COMPLETED=2  # hooray
uint32 STATE_FAILED=3     # oh no

# a brief summary of the current status of the task, for UI's
string status

# submission_time is when the task was submitted to rmf_core
builtin_interfaces/Time submission_time

# when rmf_core actually began processing the task
builtin_interfaces/Time start_time

# When this message is a summary of an in-process task, the end_time field is
# an estimate. When this message is a summary of a completed or failed task,
# end_time is the actual time.
builtin_interfaces/Time end_time
