# This message is published by the Task Dispatcher node to notify all
# Fleet Adapters to participate in a bidding process for a new task.
# Fleet Adapters may then submit a BidProposal message with their best proposal
# to accommodate the new task.

# Details of the new task
TaskProfile task_profile

# Duration for which the bidding is open
builtin_interfaces/Duration time_window
