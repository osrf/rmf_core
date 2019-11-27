# This is a template message for all types of workcells to build off from,
# which allows generic workcell libraries to have access to common message
# fields.

builtin_interfaces/Time time

# A unique ID for this request
string request_guid

# The unique ID of the workcell that this request is aimed at
string target_guid

# below are custom workcell-type message fields
