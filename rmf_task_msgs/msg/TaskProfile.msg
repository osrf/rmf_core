# Unique ID to differentiate the task
string task_id

# submission_time is when the task was first submited
builtin_interfaces/Time submission_time

# e.g. Delivery, Station.....
TaskType type  # enum here?

# is this a priority
bool priority

# Params
BehaviorParameter[] task_params
