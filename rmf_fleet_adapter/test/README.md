# Testing read\_only\_fleet\_adapter

The read_only adapter is tested for four scenarios:
1. `test_insert` : the first FleetState message is sent to the adapter which 
should create an entry in its ScheduleEntries and hence triggering a submit trajectory 


Testing the read_only_fleet_adapter requires 5 terminals 


1. Start the rmf_schedule node 
```ros2 run rmf_traffic_ros2 rmf_traffic_schedule```

2. Start the r[mf_schedule_visualizer](https://github.com/osrf/rmf_schedule_visualizer) node to print out trajectory data in a mirror. 
```ros2 run rmf_schedule_visualizer schedule_visualizer -n viz```

3. Start the read_only fleet adapter node
```ros2 run rmf_fleet_adapter read_only -f fleet1```

4. Start the testing node 
```ros2 run rmf_fleet_adapter test_read_only_adapter```

5. The tests
