
# The sequence key that refers to this table. This is equivalent to
# [to_accommodate..., for_participant]
uint32[] sequence

# Status of the table
uint8 status
uint8 STATUS_UNKNOWN = 0
uint8 STATUS_FINISHED = 1
uint8 STATUS_FORFEITED = 2
uint8 STATUS_REJECTED = 4
uint8 STATUS_ONGOING = 8
uint8 STATUS_DEFUNCT = 16

# depth in tree of nodes used for sorting
int32 depth

# index to tables array in the NegotiationStatus message. -1 indicates no parent
int32 parent_index

# participant id of proposals. same count as proposals
uint32[]  proposals_id

# proposals. same count as proposals_id
Itinerary[] proposals

