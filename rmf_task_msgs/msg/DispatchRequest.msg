# Publish by Task Dispatcher Node
# This msg is will be received by a Fleet Adapter to initiate/cancel a task

# Selected Fleet Adapter to execute/cancel the task
string fleet_name

# Task to dispatch (should be the same as in BidNotice)
TaskProfile task_profile

# add or cancel a task
uint8 method
uint8 ADD=1     # to add a task
uint8 CANCEL=2  # to cancel and remove a task
