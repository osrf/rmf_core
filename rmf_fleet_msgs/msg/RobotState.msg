string name
string model

# task_id is copied in from the most recent Request message,
# such as ModeRequest, DestinationRequest, or PathRequest
string task_id

RobotMode mode
float32 battery_percent
Location location
Location[] path
