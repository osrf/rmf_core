builtin_interfaces/Time charger_time

uint16 CHARGER_IDLE = 1 # Charger is not occupied
uint16 CHARGER_ASSIGNED = 2 # Charger has been assigned a robot 
uint16 CHARGER_CHARGING = 3 # Charger is charging
uint16 CHARGER_ERROR = 200 # Error state.

string charger_name # Name of charger. This should be the same as the
                    # name marked on the navigation graph.
string error_msg

string charger_vendor # The vendor of the charger
string charger_model # The exact model of a charger

#List of robots its ready to recieve 
string robot_fleet 
string robot_name 
