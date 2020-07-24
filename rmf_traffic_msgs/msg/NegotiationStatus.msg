
# The version number assigned to this conflict
uint64 conflict_version

# The IDs of the participants that are in conflict.
uint32[] participants

# Names of participants in the conflict. Same array size as participants
string[] participant_names

# Array of all tables
NegotiationStatusTable[] tables

