# This message is published by a Fleet Adapter in response to a BidNotice
# message.

# The name of the Fleet Adapter publishing this message
string fleet_name

# Details of the task to accommodate. This should math the TaskProfile in the
# BidNotice
TaskProfile task_profile

# The overall cost of task assignments prior to accommodating the new task
float64 prev_cost

# The overall cost of task assignments after accommodating the new task
float64 new_cost

# The estimated finish time of the new task
builtin_interfaces/Time finish_time

# The name of the robot in the fleet which will potentially execute the task
string robot_name
