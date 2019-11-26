builtin_interfaces/Time time

# A unique ID for the request, this result is for
string request_guid

# The unique ID of the dispenser that this result was sent from
string source_guid

uint8 status
uint8 ACKNOWLEDGED=0
uint8 SUCCESS=1
uint8 FAILED=2

# below are custom workcell message fields
