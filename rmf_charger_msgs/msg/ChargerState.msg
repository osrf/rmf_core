builtin_interfaces/Time charger_time

uint16 CHARGER_IDLE = 1      # Charger is not occupied
uint16 CHARGER_ASSIGNED = 2  # Charger has been assigned a robot
uint16 CHARGER_CHARGING = 3  # Charger is charging
uint16 CHARGER_ERROR = 200   # Error state, see error_msg for info

uint16 state         # Describes the current state of the said charger

string charger_name  # Name of charger. This should be the same as the
                     # name marked on the navigation graph.
string error_msg

# description of this charger
string charger_vendor
string charger_model

# the robot that is currently assigned to this charger (if any)
string robot_fleet
string robot_name
