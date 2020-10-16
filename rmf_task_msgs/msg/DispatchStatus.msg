# Publish by a action server (Fleet Adapter) in a on-change event mannner.
# Describes the task status of a on-going task

# Note: This is similar to the TaskSummary.msg (Might be a substitute)

# Fleet Adapter name
string fleet_name

TaskProfile task_profile

uint8 state
uint8 PENDING=0
uint8 ACTIVE_QUEUED=1
uint8 ACTIVE_EXECUTING=2
uint8 TERMINAL_COMPLETED=3  # hooray
uint8 TERMINAL_FAILED=4     # oh no
uint8 TERMINAL_CANCELED=5

# Allocated robot name
string robot_name 

# when rmf_core actually began processing the task
builtin_interfaces/Time start_time

# When this message is a summary of an in-process task, the end_time field is
# an estimate. When this message is a summary of a completed or failed task,
# end_time is the actual time.
builtin_interfaces/Time end_time

# a brief summary of the current status of the task, for UI's
string status
