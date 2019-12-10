string fleet_name
string robot_name
RobotMode mode

# task_id must be copied into future RobotState messages
string task_id

# Some mode changes require parameters. For example, the name of a dock.
ModeParameter[] parameters
