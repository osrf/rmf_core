# Publish by Task Dispatcher Node
# This msg is will be received by Fleet Adapter to initiate/cancel a task
# a DispatchRequest will be provided when the task is completed

# Selected Fleet Adapter / Target server
string server_id

# if task is dispatched via bidding, this should be the same
Task task

# add or cancel a task
uint8 method
uint8 CHECK=0   # to check a task
uint8 ADD=1     # to add a task
uint8 CANCEL=2  # to cancel and remove a task

# Behavior # ?? todo
