
# An array of the current blockade statuses which describe the most recent
# information
BlockadeStatus[] statuses

# This will be true when the blockade moderator has identified a gridlock that
# cannot be undone. This should never happen if a system is setup correctly. But
# it may happen if a robot is given a path whose first or last checkpoint is in
# conflict with the path of another robot.
bool has_gridlock
