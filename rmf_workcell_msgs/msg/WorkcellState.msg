# This is a template message for all types of workcells to build off from,
# which allows generic workcell libraries to have access to common message
# fields.

builtin_interfaces/Time time

# A unique ID for this workcell
string guid

# Different basic modes that the workcell could be in
int32 mode
int32 IDLE=0
int32 BUSY=1
int32 OFFLINE=2

# Queued up requests that are being handled by this workcell
string[] request_guid_queue

# below are custom workcell-type message fields
