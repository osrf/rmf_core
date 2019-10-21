
# We support 256 different profiles per Trajectory. If a trajectory instance
# needs more than 256 different unique profiles, then it needs to be split into
# multiple VehicleTrajectory messages. However that would be an extremely
# bloated Trajectory.
uint8 profile_index

# The time that this trajectory segment finishes.
# Currently this is represented by nanoseconds since the UNIX epoch.
# TODO(MXG): Consider changing to builtin_interfaces/Time
int64 final_time

# This is a 2D homogeneous position which mixes 2 translation coordinates (x, y)
# with 1 rotation coordinate (yaw).
#
# The position that the vehicle will arrive at when this trajectory segment
# ends.
float64[3] final_position

# This is a 2D homogeneous screw velocity with 2 translational components (x, y)
# and 1 rotational component (yaw).
#
# The velocity that this vehicle will end with when this trajectory segment
# ends.
float64[3] final_velocity
