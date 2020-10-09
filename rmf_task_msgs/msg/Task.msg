# Unique ID to differentiate the task
TaskProfile task_profile

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
string robot_name 
# todo: phase?

# state
uint8 state
uint8 INVALID=0
uint8 ACTIVE_QUEUED=1
uint8 ACTIVE_EXECUTING=2
uint8 TERMINAL_COMPLETED=3  # hooray
uint8 TERMINAL_FAILED=4     # oh no
uint8 TERMINAL_CANCELED=5
