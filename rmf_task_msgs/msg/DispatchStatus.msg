# Publish by a action server (Fleet Adapter) in a periodic manner.
# Describes all on-going tasks which are handled by the server

# server which the msg is published from
string server_id

# On-going queued tasks
Task[] tasks
