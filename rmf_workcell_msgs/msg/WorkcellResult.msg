# This is a template message for all types of workcells to build off from,
# which allows generic workcell libraries to have access to common message
# fields.

builtin_interfaces/Time time

# A unique ID for the request which this result is for
string request_guid

# The unique ID of the workcell that this result was sent from
string source_guid

# Different basic result statuses
uint8 status
uint8 ACKNOWLEDGED=0
uint8 SUCCESS=1
uint8 FAILED=2

# below are custom workcell message fields
