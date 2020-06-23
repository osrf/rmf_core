^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_traffic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-23)
------------------
* Merge pull request `#111 <https://github.com/osrf/rmf_core/issues/111>`_ from osrf/develop
  First Release Candidate
* Merge remote-tracking branch 'origin/develop' into fix/memory_leaks
* Cleaning up TODOs
* Tweaks to address various issues
* Fix an off-by-one error in Route Validator generation
* Remove debug output
* New adapter seems to be working
* Significant behavioral improvements
* Debugging the new fleet adapter
* Implementing a backwards compatible adapter
* Resolve merge
* Fixed planning service bug
* Lots of debug output
* Merge pull request `#105 <https://github.com/osrf/rmf_core/issues/105>`_ from osrf/develop
  New traffic scheduling and negotiation system
* RMF Linter
* Merge pull request `#107 <https://github.com/osrf/rmf_core/issues/107>`_ from osrf/bug/fix_alternative_iteration
  Fix alternative iteration in negotiations
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Adding optional waypoint names feature to the Graph class
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Fleshing out rmf_fleet_adapter public API
* Use shared_ptr instead of raw pointers to accommodate async lifecycles
* Merge branch 'bug/account_for_map_name' into feature/rxcpp
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Finish test_Negotiate
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Introduce saturation limit to skip impossible planning problems
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Small tweaks
* Negotiation tests are working
* Testing negotiations with rejections
* Fixing race conditions
* Confusing segfault in services::Negotiate
* Use shared_ptr to Responder instead of reference
* Implementing the Negotiate service
* More clear semantics
* Fix the way alternatives are iterated over during negotiation
* Fix iteration over alternatives
* Need to debug negotiation room
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Merge pull request `#106 <https://github.com/osrf/rmf_core/issues/106>`_ from osrf/feature/remove_plan_cruft
  Remove cruft (unnecessary motions) from plans
* Filling in implementation of FindEmergencyPullover
* Fix merge error
* Fix merge
* Remove unnecessary clock checks
* Remove cruft (unnecessary motions) from plans
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Adding parking spot flag to waypoints
* Make sure valid alternatives are always returned
* Redefining what a holding point is
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Filling out the implementation of GoToPlace
* Continuing implementation of GoToPlace
* Adding Planner::setup()
* implement a stubborn negotiator
* Small tweaks
* Have the plan start computation account for the map name
* Fix merge conflicts
* Fix merge conflicts (`#102 <https://github.com/osrf/rmf_core/issues/102>`_)
* Complete traffic negotiation system (`#101 <https://github.com/osrf/rmf_core/issues/101>`_)
* Specify maximum estimate for planner and use this to discard undesirable planning tasks
* Merge in latest changes
* Cleanup
* Fix implementation bugs (`#100 <https://github.com/osrf/rmf_core/issues/100>`_)
* initial rxcpp poc
* Finished updating fleet adapters -- need to track down bad optional access
* Updating the API for Negotation
* Add Snapshot and Snappable interface classes
* Tweak Negotiator API and fix bug
* Refactoring Timeline and Negotiation::Table to allow thread-safe snapshots and views
* Move the SimpleNegotiator debugging printout to Debug
* Remove unnecessary comments
* Edge cases are confirmed to be fixed
* All tests are passing once again
* Creating a problem skeleton to test multi-threading
* Add time information to DetectConflict
* Add end caps to NegotiatingRouteValidator
* Add test for resuming
* Refactor planning implementation so that planning can be resumed after an interruption
* Get test_Negotiator working again
* Strange bug in NegotiatingRouteValidator
* Update the definition of Negotiator::Responder
* Rollout scheme appears to be working
* Adding Rollout test and eliminating redundant rollouts
* Refactor Plan and implement Rollout class
* Allow the planner to pause at the starting location
* Allow planners to pause a vehicle at its starting location
* Filling in implementation for negotiation rollout alternatives
* Reworking the API for route validators
* Implementing rollout alternative negotiation
* Merge branch 'merge/lenient_vicinity' into test/edge_cases
* Fix style
* Merging in lenient vicinity changes
* Finish implementation of Planner debugging
* Beginning implementation of Planner debugging tools
* Creating a Planner::Debug API
* Addressing multi-participant negotiation issues (`#95 <https://github.com/osrf/rmf_core/issues/95>`_)
* some packages dependencies fixes (`#94 <https://github.com/osrf/rmf_core/issues/94>`_)
* RMF Linter (`#96 <https://github.com/osrf/rmf_core/issues/96>`_)
* Testing multiple participant negotiation (`#90 <https://github.com/osrf/rmf_core/issues/90>`_)
* Improve fleet adapter (`#92 <https://github.com/osrf/rmf_core/issues/92>`_)
* Tests for the Profile class (`#86 <https://github.com/osrf/rmf_core/issues/86>`_)
  * Tests for profile
  * Added tests
  * More tests
  * Corrected test assumptions
  * Updated tests
  * Update rmf_traffic/test/unit/test_Profile.cpp
  Co-Authored-By: Grey <grey@openrobotics.org>
  Co-authored-by: Grey <grey@openrobotics.org>
* Migrate fleet adapter to use the schedule participant system (`#84 <https://github.com/osrf/rmf_core/issues/84>`_)
* Test/modular (`#82 <https://github.com/osrf/rmf_core/issues/82>`_)
  * Added tests for Modular
  * Removed unnecessary header
  * Removed whitespace
  * Fixed undefined behavior from increment
  * Added const to value declarations
  * Update rmf_traffic/test/unit/schedule/test_Modular.cpp
  Co-Authored-By: Grey <grey@openrobotics.org>
  * Make tests more explicit (`#85 <https://github.com/osrf/rmf_core/issues/85>`_)
  Co-authored-by: Grey <grey@openrobotics.org>
* Updated dependencies (`#80 <https://github.com/osrf/rmf_core/issues/80>`_)
* Schedule Participants (`#77 <https://github.com/osrf/rmf_core/issues/77>`_)
* Feature/broad phase refinement (`#68 <https://github.com/osrf/rmf_core/issues/68>`_)
  * added function to compute coefficients of cubic spline motions
  * calculate bounding boxes for trajectories
  * check for overlap working
  * fixed failing tests after braod_phase modification
  * boundary values are evaluated for local extrema
  * Broad-phase checks for overlap of bounding boxes of segments with overlapping time ranges
  * Fixed failing test after broad_phase() update
  * Formatting fixes in test_Conflict.cpp
  * Removed commented code and unsued functions
  * Removed commented line from CMakeLists.txt
  * Removed unnecessary headers
  * Removed space
  * Refactored broad_phase()
  * Added get_characteristic_length() to FinalShape
  * Fixed usage of get_characteristic_length() in broad_phase()
  * Removed header
  * Fixes from PR
  * Fixed extrema calculation for non-quadratic derivative of spline motion
  * Renamed t_sols to extrema_candidates
  * Fixes after second review
* Feature/refactor merge to graph (`#67 <https://github.com/osrf/rmf_core/issues/67>`_)
  * shifted api and implementation into rmf_traffic::agv::Planner, no longer rclcpp dependent, builds
  * problems returning from the optional StartSet
  * figured out how to use value of optional
  * tests should be quite complete, starting PR and cleanup
  * getting conflicted lane indices in tests
  * finished writing tests and passes
  * removed redundant planners in tests, fixed typos, added const to references
* Fix orientation calculations for plan starts (`#65 <https://github.com/osrf/rmf_core/issues/65>`_)
* Make the schedule manager persistent (`#63 <https://github.com/osrf/rmf_core/issues/63>`_)
* Allow the schedule to jump forward (`#61 <https://github.com/osrf/rmf_core/issues/61>`_)
* Add a docking event to the graph and planner (`#56 <https://github.com/osrf/rmf_core/issues/56>`_)
* Fixes (`#54 <https://github.com/osrf/rmf_core/issues/54>`_)
* Fleet adapter behavior fixes (`#51 <https://github.com/osrf/rmf_core/issues/51>`_)
* Implement a responsive full-control fleet adapter (`#46 <https://github.com/osrf/rmf_core/issues/46>`_)
* New planning start features (`#39 <https://github.com/osrf/rmf_core/issues/39>`_)
* Add replace and delay services to the schedule (`#32 <https://github.com/osrf/rmf_core/issues/32>`_)
* Fixed tests when compiling with clang (`#31 <https://github.com/osrf/rmf_core/issues/31>`_)
  Required removing auto in favor of Eigen::Vector in all variable
  declarations of test code.
* Finish and use rmf_utils::optional (`#27 <https://github.com/osrf/rmf_core/issues/27>`_)
* Add door and lift planning features (`#26 <https://github.com/osrf/rmf_core/issues/26>`_)
* Small fixes for trajectory interpolation and conversion (`#15 <https://github.com/osrf/rmf_core/issues/15>`_)
* Merge branch 'master' of ssh://github.com/osrf/rmf_core into rmf_door_msgs
* Merge pull request `#14 <https://github.com/osrf/rmf_core/issues/14>`_ from osrf/consolidation
  Feature Dump
* Merged
* Merging in the conflict tests
* Merge pull request `#10 <https://github.com/osrf/rmf_core/issues/10>`_ from osrf/ament_cmake_tests
  Added catch2 test to ament
* Merge
* Fixed mistake in test
* Some surprising performance results -- should investigate
* Finished updating tests
* Finished refactoring AGV Planner API, need to fix tests
* Continuing to refactor implementation of Planner
* Continuing to implement new planner
* Added ament_cmake_catch2 used in rmf_traffic tests
  The ament_cmake_catch2 package is based on ament_cmake_gtest, test files
  are now discovered dynamically instead of being hardcoded in the
  CMakeLists file.
* Beginning implementation of the cached DifferentialDrive planner
* Finishing the planning API
* Refactoring into a translation unit
* Major restructuring to Planning API + implementation
* Redesigning Planner class
* Bringing back more tests
* Planner working again
* WIP: attempting a new expansion strategy
* Merging up the planner tests
* temporary hacks
* Adding the ability to automatically loop test task requests
* Interfacing with Gazebo
* Registering the mirror queries of fleet adapters
* Parsing the graph and getting the mirror manager spun up
* added copy and move tests for Graph
* more Graph tests
* added Graph tests
* commented out agv_palnning tests that take long while to run to increase speed of testing. Also added test for Graph API
* Beginning tools for managing remote mirrors
* Even more message conversions
* added planner tests with multiple non-conflicting obstacles in database
* removed printed messages
* Translating more messages
* uncommented older planner tests
* updated test description
* Planner test will multiple obstacles in bigger graph
* Planner tests with larger graph
* removed printed messages
* adjusted trajectories for Planner tests
* more multi-obstacle tests for Planner
* added Planner test for multiple obstacles
* added more Planner tests
* Beginning ROS2 wrappers for rmf_traffic
* corrected inequality in is_finished() and modified API description of Planner::solve() to match output when intial and goal conditions are identical
* Jot down notes on how we may be able to improve the planner
* Merge remote-tracking branch 'origin/schedule_database_tests_for_planner' into agv_planning
* commented out failing tests and debug printouts
* commiting changes before creating new branch without failing tests
* Add obstacles to each test case
* spacetime tests fro multi-segment trajectories
* Create a situation that requires the plan to use holding point `#4 <https://github.com/osrf/rmf_core/issues/4>`_
* Schedule-respecting planner is tested and working
* Merge branch 'schedule_database_tests' into debug/agv_planning
* Testing planning against the database
* checked conflict time for mult-segment collision
* multi-segment conflict test
* Increase planning speeds by orders of magnitude
* lane exit constraints are working
* Planning implementation done -- initial tests are working
* spacetime tests for circle profiles
* minor progress
* spacetime tests with circular trajectory profiles
* Continuing implementation of AGV planner
* more spacetime conflict tests
* more spacetime conflict tests
* added checks for trajectory in Changes Patch
* Clean up debug printouts
* Debugging the query tests
* Tweak some of the APIs
* added more tests for Mirror which are currently failing
* additional spacetime tests
* spacetime conflict tests are passing
* added test to check intersection between spacetime and trajectory using DetectConflictInternal
* added test for specialized mirrors that query for regions
* Fleshing out implementation of AGV planner
* updated header
* conflict checking with mirror
* added tests for mirror
* Finish changes to agv::VehicleTraits
* fixed cull test
* Fix assumptions regarding Database::erase
* Fix version history to bring back fixes
* defined get_num_entries
* Change API VehicleTraits
* Merge branch 'schedule_database_tests' of ssh://github.com/osrf/rmf_core into schedule_database_tests
* Port over fixes
* Tweak the API of VehicleTraits
* Fix faulty assumption about erase
* added check for number of trajectories in database
* fixed the trajectory equality checker
* replaced query_everything with make-query
* fix merge conflict
* CHECK for comparing trajectories
* Adding a debug utility for the Viewer
* Merge remote-tracking branch 'origin/schedule_database_tests' into schedule_database_tests
* corrected trajectory definition
* Fix implementation for querying all changes
* Declaring the Graph::solve function
* Added basic tests for Databse member functions
* Finished design and implementation of agv::Graph building API
* merge
* Merge remote-tracking branch 'origin/schedule_database' into schedule_database_tests
* initial commit
* Fix initial cull bug
* Designing API for agv::Graph
* Add a note about Time
* modified rotating collision test
* Implement the Mirror functions
* Implemented most of the Database functions
* Use an explicitly sized integer for tracking version numbers
* Introduce class to ensure correct version comparisons
* Finish implementing change query
* added test for conflict only when robot is rotated
* Implementing Database::changes() query
* tests for trajectories in different maps
* Implementing Database::Change functions
* Implemented DeepOrShallowTrajectory
* Refactoring Viewer and beginning Database::Change implementation
* Refactor Query::Region into its own class
* Add more tests for conflicts between trajectories and regions
* Fix StaticMotion and add a simple test
* Finished implementing Trajectory conflicts against static regions
* Add a tougher conflict detection test, and tweak the FCL settings
* Add documentation
* Fix merge conflicts
* Implemented FinalShape
* Beginning to implement shape finalization
* Implementing Viewer query and introducing FinalShape class
* Implementing Viewer querying
* update nonsense variable names
* demonstrating possible broad_phase false positive
* add between function commutative check
* marginally more coherent framework
* Added APIs for interrupting and delaying schedule trajectories
* fixed CHECK_ConflictData notation and added note in utils_conflict
* temporary commit to highlight testing query
* change definition of get_segments to const
* temporary, buggy version of CHECK_ConflictData
* finish up narrow phase commutative checks
* Allow Queries to not stipulate a time range
* Merge pull request `#3 <https://github.com/osrf/rmf_core/issues/3>`_ from osrf/agv_utilities
  Utilities to generate trajectories for AGVs
* refining narrow_phase checks
* Merge remote-tracking branch 'origin/broad_phase_exception' into add_conflict_tests
* Finished API for schedule databases and mirrors
* Adding some documentation
* Introducing Database, Mirror, and Viewer classes
* Have broad_phase throw an exception for Trajectories with less than 2 elements
* refactor helper functions for commutative checks
* temporary save point
* redimentary effort one len-1 trajectories for now
* tests for empty trajectories
* Merge branch 'agv_utilities' into add_conflict_tests
* merge
* Merge pull request `#1 <https://github.com/osrf/rmf_core/issues/1>`_ from osrf/rmf_traffic_controller-prototype
  Traffic Trajectory and Conflict Detection API
* Renaming test file
* Finish design and implementation of schedule::Query and begin rudimentary tests
* modify basic tests
* update utils_trajectory
* add initial tests for empty trajectories
* Finished design and implementation of Query request class
* Merge pull request `#2 <https://github.com/osrf/rmf_core/issues/2>`_ from osrf/traffic_controller_add_tests
  Unit tests for Trajectory API
* liberal const for code sparkles
* Merge remote-tracking branch 'origin/agv_utilities' into add_conflict_tests
* Implementing a generic way of iterating
* designing and implementing schedule query class
* Merge branch 'rmf_traffic_controller-prototype' into railcar_trajectory
* Remove all trace of the ConflictResult class
* add PR changes
* refactor to remove namespace shortcuts
* Adding tests for points that are close to each other
* Created tests for distant points
* Organize unit tests
* Rename the track namespace to agv
* Fixing issues caught by tests
* cleanup flags
* cleanup
* add tests
* Merge branch 'refactor_traffic_controller_add_tests' of github.com:osrf/rmf_core into refactor_traffic_controller_add_tests
* add refactored tests, rest to merge with workstation at work
* Beginning to write tests for interpolation
* added more tests: remains trajectory functions
* add more refactoring
* refactor profile unit tests
* Computing constant-acceleration interpolations based on positions
* refactor profile generator
* add cleaner trajectory creation function
* start refactor
* Creating interpolation API
* moar tests
* Restructure the src filesystem a bit
* Tweaking APIs and adding an API for specifying simple vehicle properties
* add move semantic checks for base_iterator
* Clean up user files
* Use sqrt instead of typing out a float
* Mention a useful website
* Add a wishlist of tests
* Finished implementation of DetectConflict and added rudimentary tests
* Rename Segment functions and finish DetectConflict implementation
* Almost done implementing DetectConflict
* Finished Spline class
* Merge in Boon Han's tests and fix time adjustment bug
* copy new files over from branch
* Fix build error
* modify test to avoid time conflict
* add tests
* add tests
* refactor tests for readability
* Revert "Designing the API for detecting conflicts"
  This reverts commit 47ebe60320205a534bbc0e862aa4e5880f694f89.
* Merge remote-tracking branch 'origin/rmf_traffic_controller-prototype' into traffic_controller_add_tests
* Designing the API for detecting conflicts
* getting the hang of testing framework
* Merge branch 'rmf_traffic_controller-prototype' of github.com:osrf/rmf_core into traffic_controller_add_tests
* Add functions for checking the time range of a Trajectory
* Wrote tests and fixed bugs for copying
* export compile commands
* Add some tests and fix some bugs
* Fix the constructor of Trajectory
* Just remembered the begin() function
* Finished implementing the Trajectory class -- needs testing
* wip
* wip
* Change Trajectory Segment storage implementation to use std::list
* Reworking Segment storage implementation to avoid invalidating iterators
* Add Catch2 testing utilities
* Implementing Trajectory Segment functions
* Designing Trajectory Segment and iterator API
* Implemented Trajectory::Profile
* Refactoring rmf_traffic_controller into rmf_traffic
* Contributors: Aaron Chong, Boon Han, Charayaphan Nakorn Boon Han, Grey, Luca Della Vedova, Marco A. Gutiérrez, Michael X. Grey, Morgan Quigley, Yadu, Yadunund, koonpeng
