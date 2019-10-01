uint32 segment_type
uint32 SEGMENT_TYPE_PLANAR=0
uint32 SEGMENT_TYPE_VERTICAL=1

# fields used when segment_type == SEGMENT_TYPE_PLANAR
rmf_schedule_msgs/VehicleTrajectoryPoint[] planar_points
string planar_map_name

# fields used when segment_type == SEGMENT_TYPE_VERTICAL
string vertical_starting_map
string vertical_ending_map
rmf_schedule_msgs/VehicleTrajectoryPoint[] vertical_lift_entry_points
rmf_schedule_msgs/VehicleTrajectoryPoint[] vertical_lift_exit_points
