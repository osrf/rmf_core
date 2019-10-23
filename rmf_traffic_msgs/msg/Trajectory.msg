
# The names of the maps to perform collision detection on for this robot.
string[] maps

# A Trajectory is comprised of multiple Segments, each of which represents a
# cubic spline interpolated from the previous segment. The very first segment in
# the sequence represents the initial conditions of the trajectory.
TrajectorySegment[] segments

# The profiles that are available for the segments to use. Mutliple segments may
# share a profile, so we store them here to avoid redundancy.
TrajectoryProfile[] profiles

# The convex shapes that are available for the profiles to use
ConvexShapeContext convex_shape_context

# TODO(MXG): Consider moving convex_shape_context up a level in the message
# stack (e.g. SubmitTrajectory) so that it can be shared by multiple Trajectory
# messages. Also consider making a TrajectoryContext message that contains the
# TrajectoryProfile[] data and the ConvexShapeContext.
