# This message is published by Task Dispatcher Node to either award or cancel a
# task for a Fleet Adapter

# The selected Fleet Adapter to award/cancel the task
string fleet_name

# The details of the task to be awarded or cancelled. This should match the
# TaskProfile in the corresponding BidNotice message
TaskProfile task_profile

# Add or Cancel a task
uint8 method
uint8 ADD=1     # to add a task
uint8 CANCEL=2  # to cancel and remove a task
