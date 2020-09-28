# Publish by an action server (Fleet Adapter) 
# This will be published when a task request msg is 
# received or a task is completed.

# server which the msg is published from
string server_id

# Completed task
Task task

uint8 response
uint8 REJECTED=0
uint8 ACCEPTED=1
uint8 TERMINATED=2

# ============================================================================
# below will only be valid when task/response is TERMINATED

uint8 terminal_state
uint8 INVALID=0
uint8 COMPLETED=1  # hooray
uint8 FAILED=2     # oh no
uint8 CANCELED=3
