# task_id is intended to be a pseudo-random string generated
# by the caller which can be used to identify this task as it
# moves between the queues to completion (or failure).
string task_id

# robot_type can be used to specify a particular robot fleet
# for this request
string robot_type

# The number of times the robot should loop between the specified points.
uint32 num_loops

# The name of the waypoint where the robot should begin its loop. If the robot
# is not already at this point, it will begin the task by moving there.
string start_name

# The name of the waypoint where the robot should end its looping. The robot
# will visit this waypoint num_loops times and then stop here on the last
# visit.
string finish_name
