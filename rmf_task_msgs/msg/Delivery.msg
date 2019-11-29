# task_id is intended to be a pseudo-random string generated
# by the caller which can be used to identify this task as it
# moves between the queues to completion (or failure).
string task_id

rmf_dispenser_msgs/DispenserRequestItem[] items

string pickup_place_name
Behavior pickup_behavior

string dropoff_place_name
Behavior dropoff_behavior
