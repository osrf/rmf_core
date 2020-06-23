^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_traffic_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-23)
------------------
* Merge pull request `#111 <https://github.com/osrf/rmf_core/issues/111>`_ from osrf/develop
  First Release Candidate
* Get rid of unnecessary warnings
* Merge remote-tracking branch 'origin/develop' into fix/memory_leaks
* Rename Negotiation messages
* Cleaning up TODOs
* Tweaks to address various issues
* Add some assertions and remove a useless warning
* Remove debug output
* Significant behavioral improvements
* Debugging the new fleet adapter
* Fix enable_shared_from_this bugs
* Resolve merge
* Merge pull request `#105 <https://github.com/osrf/rmf_core/issues/105>`_ from osrf/develop
  New traffic scheduling and negotiation system
* RMF Linter
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Use shared_ptr instead of raw pointers to accommodate async lifecycles
* Use shared_ptr to Responder instead of reference
* Fix merge conflicts
* Fix merge conflicts (`#102 <https://github.com/osrf/rmf_core/issues/102>`_)
* Complete traffic negotiation system (`#101 <https://github.com/osrf/rmf_core/issues/101>`_)
* Merge in latest changes
* Fix implementation bugs (`#100 <https://github.com/osrf/rmf_core/issues/100>`_)
* Finished updating fleet adapters -- need to track down bad optional access
* Updating the API for Negotation
* Add Snapshot and Snappable interface classes
* Merge branch 'merge/lenient_vicinity' into test/edge_cases
* Fix style
* Merging in lenient vicinity changes
* some packages dependencies fixes (`#94 <https://github.com/osrf/rmf_core/issues/94>`_)
* RMF Linter (`#96 <https://github.com/osrf/rmf_core/issues/96>`_)
* Improve fleet adapter (`#92 <https://github.com/osrf/rmf_core/issues/92>`_)
* Migrate fleet adapter to use the schedule participant system (`#84 <https://github.com/osrf/rmf_core/issues/84>`_)
* Updated dependencies (`#80 <https://github.com/osrf/rmf_core/issues/80>`_)
* Changes that were used for DP2 (`#64 <https://github.com/osrf/rmf_core/issues/64>`_)
* Allow the schedule to jump forward (`#61 <https://github.com/osrf/rmf_core/issues/61>`_)
* Fixes (`#54 <https://github.com/osrf/rmf_core/issues/54>`_)
* Fleet adapter behavior fixes (`#51 <https://github.com/osrf/rmf_core/issues/51>`_)
* Implement a responsive full-control fleet adapter (`#46 <https://github.com/osrf/rmf_core/issues/46>`_)
* Make sure mirrors are woken up for all schedule modifications (`#37 <https://github.com/osrf/rmf_core/issues/37>`_)
* Add replace and delay services to the schedule (`#32 <https://github.com/osrf/rmf_core/issues/32>`_)
* Read only fleet adapter (`#29 <https://github.com/osrf/rmf_core/issues/29>`_)
* Integrate read-only fleet adapter (`#18 <https://github.com/osrf/rmf_core/issues/18>`_)
* added cmake install stuff from integrate_generic_fleet_adapter (`#16 <https://github.com/osrf/rmf_core/issues/16>`_)
* Small fixes for trajectory interpolation and conversion (`#15 <https://github.com/osrf/rmf_core/issues/15>`_)
* Merge branch 'master' of ssh://github.com/osrf/rmf_core into rmf_door_msgs
* Merge pull request `#14 <https://github.com/osrf/rmf_core/issues/14>`_ from osrf/consolidation
  Feature Dump
* eliminate dependency on rmf_msgs
* Remove dependency on rmf_msgs
* Use smaller radius for more impressive demo
* temporary hacks
* Adding the ability to automatically loop test task requests
* Interfacing with Gazebo
* Generating and scheduling test plans
* Registering the mirror queries of fleet adapters
* Remember to launch the MirrorManagerFuture's discovery thread
* Parsing the graph and getting the mirror manager spun up
* Beginning prototype of a fleet adapter
* Design and implement ROS2 MirrorManager
* Converting Spacetime Query component
* Beginning tools for managing remote mirrors
* Even more message conversions
* Translating more messages
* implement first draft of schedule submissions and schedule querying
* Parse Trajectory message into trajectory instance
* Beginning ROS2 wrappers for rmf_traffic
* Contributors: Aaron Chong, Grey, Marco A. Gutiérrez, Michael X. Grey, Morgan Quigley, Yadu, Yadunund, koonpeng
