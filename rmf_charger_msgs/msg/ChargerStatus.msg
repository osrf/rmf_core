builtin_interfaces/Time charger_time

uint16 IDLE = 1 # Charger is not occupied
uint16 CHARGING = 2 # Charger is charging
uint16 ERROR = 200 # Error state.

string charger_name
string error_msg