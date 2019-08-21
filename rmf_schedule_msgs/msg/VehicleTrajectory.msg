string vehicle_guid

# a Trajectory is comprised of multiple Segments, each of which is either
#   planar: driving around on a particular map
#   vertical: entering a lift/elevator, riding to another map, and exiting
rmf_schedule_msgs/VehicleTrajectorySegment[] segments

# bounding area for the footprint of the robot
float32[2] footprint_dimensions
uint8 footprint_shape
uint8 FOOTPRINT_SHAPE_RECTANGLE=0
uint8 FOOTPRINT_SHAPE_CIRCLE=1

# summary of the vehicle navigation: will it stick to the proposed path
# exactly, or will it "wander" somewhat if it encounters an obstacle?
uint8 navigation_style
uint8 NAVIGATION_STYLE_RAILWAY=0
uint8 NAVIGATION_STYLE_DYNAMIC=1
