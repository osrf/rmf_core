
# The time when this Waypoint should be reached
int64 time

# This is a 2D homogeneous position which mixes 2 translation coordinates (x, y)
# with 1 rotation coordinate (yaw).
#
# The position of this Waypoint
float64[3] position

# This is a 2D homogeneous screw velocity with 2 translational components (x, y)
# and 1 rotational component (yaw).
#
# The velocity that this vehicle should have when it reaches this Waypoint
float64[3] velocity
