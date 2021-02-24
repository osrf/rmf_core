# task_id is intended to be a pseudo-random string generated
# by the caller which can be used to identify this task as it
# moves between the queues to completion (or failure).
string task_id

string object_type

bool is_object_id_known
string object_id

string pickup_place_name

bool is_dropoff_place_known
string dropoff_place_name
