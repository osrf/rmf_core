
# The conflict ID that this proposal is targeted at
uint64 conflict_version

# The version number for this proposal within the negotiation
uint64 proposal_version

# The participant ID that this proposal is coming from
uint64 for_participant

# The participant IDs that this proposal is trying to accommodate. As each
# participant proposes their ideal itinerary, the other participants in the
# conflict will propose itineraries which accommodate it.
#
# The order of IDs in this dynamic array have important semantic meaning about
# which itineraries are being accommodated. For example:
#
# [] (empty):  This proposal does not accommodate any other participants. This
#              is the best possible itinerary for this participant.
#
# [3]:         This proposal is the best itinerary that can accommodate the
#              ideal itinerary of participant 3.
#
# [3, 7]:      This proposal is the best itinerary for this participant that can
#              accommodate both the ideal itinerary of participant 3 and the
#              best itinerary of participant 7 that accommodates the ideal
#              itinerary of participant 3.
#
# [3, 7, ...]: This proposal is the best itinerary that can accommodate the
#              ideal itinerary of participant 3 and the best itineraries that
#              accommodate the best itineraries of the participants that precede
#              them in the list, recursively.
ScheduleConflictKey[] to_accommodate

# The itinerary that is being proposed for this participant
Route[] itinerary
