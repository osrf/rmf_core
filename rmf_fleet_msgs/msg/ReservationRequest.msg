string fleet_name
string robot_name

# determines the type of reservation you are making
string RESERVATION_PARKING="parking"
string RESERVATION_CHARGING="charging"
string reservation_type

uint64[] locations

builtin_interfaces/Time time
builtin_interfaces/Duration duration

# task_id must be copied into future RobotState messages
string task_id
