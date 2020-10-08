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
uint8 active_state
uint8 INVALID=0
uint8 QUEUED=1
uint8 EXECUTING=2
uint8 CANCELING=3
