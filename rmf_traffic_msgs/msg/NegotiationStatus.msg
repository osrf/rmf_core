
# The version number assigned to this conflict
uint64 conflict_version

# The IDs of the participants that are in conflict.
uint32 participant

# Names of participants in the conflict. Same array size as participants
string participant_name

uint8 status
uint8 STATUS_UNKNOWN = 0
uint8 STATUS_FINISHED = 1
uint8 STATUS_FORFEITED = 2
uint8 STATUS_REJECTED = 4
uint8 STATUS_ONGOING = 8
uint8 STATUS_DEFUNCT = 16

# The sequence key that refers to this table. This is equivalent to
# [to_accommodate..., for_participant]
uint32[] sequence

# proposals. same count as proposals_id
Itinerary[] proposals
