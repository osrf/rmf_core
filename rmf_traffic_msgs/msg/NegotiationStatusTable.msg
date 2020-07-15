
# The sequence key that refers to this table. This is equivalent to
# [to_accommodate..., for_participant]
uint64[] sequence

# indices to table array of NegotiationStatus.msg
uint32[] children

# Finished status
bool finished

# Forfeit status
bool forfeited

# Rejected status
bool rejected

# Ongoing status
bool ongoing

# Defunct status
bool defunct

# depth in tree of nodes used for sorting
int32 depth

# index to tables array in the NegotiationStatus message. -1 indicates no parent
int32 parent_index

# participant id of proposals. same count as proposals
uint64[]  proposals_id

# proposals. same count as proposals_id
Itinerary[] proposals

