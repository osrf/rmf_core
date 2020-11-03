# Unique ID to differentiate the task
string task_id

# Task submission time
builtin_interfaces/Time submission_time

# Desired start time of a task
builtin_interfaces/Time start_time

# Task type
uint32 task_type
uint32 TASK_TYPE_STATION=0
uint32 TASK_TYPE_LOOP=1
uint32 TASK_TYPE_DELIVERY=2
uint32 TASK_TYPE_CHARGING=3
uint32 TASK_TYPE_CLEANING=4
uint32 TASK_TYPE_PATROL=5

# Task definitions
Station station
Loop loop
Delivery delivery
# Charge charge
Clean clean
# Patrol patrol

