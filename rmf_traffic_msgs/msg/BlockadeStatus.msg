
# The Participant ID that this status is for
uint64 participant

# The latest reservation known for this participant
uint64 reservation

# This is true if and only if the moderator has ever received a ready notice
# from the participant
bool any_ready

# If any_ready is true, then this is the most recent ready checkpoint that the
# moderator knows about. If any_ready is false, then this field is meaningless.
uint64 last_ready

# The last checkpoint that the moderator knows of the participant reaching
uint64 last_reached

# The first checkpoint that's currently blockaded for this participant
uint64 assignment_begin

# The last checkpoint that's currently blockaded for this participant
uint64 assignment_end
