builtin_interfaces/Time charger_time

uint16 CHARGER_IDLE = 1 # Charger is not occupied
uint16 CHARGER_CHARGING = 2 # Charger is charging
uint16 CHARGER_ERROR = 200 # Error state.

string name # Name of charger. This should be the same as the
            # name marked on the navigation graph.
string error_msg

string vendor # The vendor of the charger
string model # The exact model of a charger

string[] ready_to_recieve #List of robots its ready to recieve 