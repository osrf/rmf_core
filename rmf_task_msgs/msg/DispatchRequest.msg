# Publish by Task Dispatcher Node
# This msg is will be received by Fleet Adapter to initiate/cancel a task
# a DispatchRequest will be provided when the task is completed

# Selected Fleet Adapter server name
string fleet_name

# if task is dispatched via bidding, this should be the same
TaskProfile task_profile

# add or cancel a task
uint8 method
uint8 ADD=1     # to add a task
uint8 CANCEL=2  # to cancel and remove a task
