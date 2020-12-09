
# The participant whose reservation is being canceled
uint64 participant

# True if all reservations for this participants should be considered cancelled
bool all_reservations

# If all_reservations is false, then this is the last reservation that should
# be considered cancelled. If all_reservations is true, then this field is
# meaningless
uint64 reservation
