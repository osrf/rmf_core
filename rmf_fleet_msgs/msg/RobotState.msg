string name
string model

# task_id is copied in from the most recent Request message,
# such as ModeRequest, DestinationRequest, or PathRequest
string task_id

# The sequence number of this message. Every new message should increment the
# sequence number by 1.
uint64 seq

RobotMode mode
float32 battery_percent
Location location
Location[] path
