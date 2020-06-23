^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_fleet_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-23)
------------------
* Merge pull request `#115 <https://github.com/osrf/rmf_core/issues/115>`_ from osrf/bug/avoid_race_conditions
  Avoid a possible source of race conditions by using mutex
* Avoid a possible source of race conditions by using mutex
* Remove an incorrect line of documentation
* Merge pull request `#111 <https://github.com/osrf/rmf_core/issues/111>`_ from osrf/develop
  First Release Candidate
* Merge pull request `#112 <https://github.com/osrf/rmf_core/issues/112>`_ from osrf/fix/memory_leaks
  Fix/memory leaks
* Fix subscription and compilation error
* Merge remote-tracking branch 'origin/develop' into fix/memory_leaks
* fix: lambda leaks
* Cleaning up TODOs
* Remove debug printout
* Recreate memory leak in unit test
* Merge pull request `#108 <https://github.com/osrf/rmf_core/issues/108>`_ from osrf/test/use-robot-context
  refactor phases and tests to use RobotContext
* Fix delivery rejection
* Fix queuing bug
* Tweaks to address various issues
* remove bad assertions
* Better assertions
* Add some assertions and remove a useless warning
* Merge pull request `#109 <https://github.com/osrf/rmf_core/issues/109>`_ from methylDragon/develop
  Update CMakeLists
* Fix CMakeLists
* refactor phases and tests to use RobotContext
* Remove debug output
* New adapter seems to be working
* Significant behavioral improvements
* Debugging the new fleet adapter
* Fix enable_shared_from_this bugs
* Fix typo
* Finished backwards compatible adapter -- needs testing
* Implementing a backwards compatible adapter
* Add a timeout to the planning services
* Resolve merge
* Add phase count to task status
* Update status when planning begins
* Fixed planning service bug
* Lots of debug output
* use different rclcpp contexts per test
* Added test for loop request
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Add loop request and fix MoveRobot waypoints
* always capture "this" via weak_ptr
  fixes lifetime errors
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Refactor use of enable_shared_from_this
* Fix Task
* Debugging weird task behavior
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Test Task Summary publications
* Remove unnecessary includes
* fix phase wrongly reported as completed when state messages arrive late
* Reduce obvious memory leaks
* Clean up debugging
* All bugs seems to be addressed -- needs cleanup
* Merge pull request `#105 <https://github.com/osrf/rmf_core/issues/105>`_ from osrf/develop
  New traffic scheduling and negotiation system
* RMF Linter
* Tweak test
* More debug printout
* Debugging MoveRobot
* use nullptr initial value for state as well
* fix phase hanging due to missing dispenser result
* fix phase not completing
* Further debugging
* Attempting to debug the GoToPlace phase
* Writing delivery test
* Finish delivery task implementation
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Implementing delivery tasks
* remove boost dependency
* DispenseItem also succeed when the request is acknowedged and no longer in queue
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* reports status when started and completed
* tests and fixes
* Finish delivery estimation function
* Adding optional waypoint names feature to the Graph class
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Fleshing out fleet adapter internals to make it testable
* Continuing implementation of public API
* dock robot phase
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* fix door phase logic
* Fleshing out rmf_fleet_adapter public API
* Use shared_ptr instead of raw pointers to accommodate async lifecycles
* Merge branch 'bug/account_for_map_name' into feature/rxcpp
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Add worker and transport to the robot context
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Count unique status messages; ignore redundant waiting messages
* Fix test_FindEmergencyPullover
* Finish test_Negotiate
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Remove debugging cruft
* Introduce saturation limit to skip impossible planning problems
* Experiencing issues with multithreaded planning
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* moar tests; stablized (hopefully) some flaky tests
* Small tweaks
* add more tests; some fixes
* Negotiation tests are working
* Testing negotiations with rejections
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Fixing race conditions
* Fix at least one cause of segfaults
* Eliminate one source of segfaults
* Confusing segfault in services::Negotiate
* refactor for more DRY-ness, add cancellation to MoveRobot
* Finish implementing Negotiate service
* Use shared_ptr to Responder instead of reference
* Implementing the Negotiate service
* Need to debug negotiation room
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Finish implementation and tests for FindPath and FindEmergencyPullover
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Shift job scheduling to the supervisor
* add note for the compiler requirements
* Tracking sync failures for the PlanPath example
* Writing tests for the planning services
* Filling in implementation of FindEmergencyPullover
* Fix merge error
* Merge branch 'feature/planning_jobs' into feature/rxcpp
* Implementing async planning jobs and services
* Implementing various planning jobs
* added more tests; fix more bugs
* use current task status instead of cancelled flag to avoid possible race condition
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* add some door open phase tests, fix some bugs
  refactor phases to take observable to shared_ptr, similar to how rclcpp subscription pass values
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Adding parking spot flag to waypoints
* Make sure valid alternatives are always returned
* keep things more DRY with helper operators; unsubscribe to source when phase completes
* Redefining what a holding point is
* actions no longer take reference
* refactor dispense item to chain observables
* refactor lift requests to chain observable instead of wrapping them
* use weak_ptr to prevent circular references
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* door request cancellations
* refactor door control action to chain observable instead of wrapping them, should give better life cycle management
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Filling out the implementation of GoToPlace
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* check supervisor heartbeat
* Do not unsubscribe from the active phase right away when canceling a task
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* use uuid for request ids
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Add heartbeat for the door supervisor
* add request lift phase
* add dispense item phase
* add move robot phase
* add door close phase
* specialized description that includes the door name
* Continuing implementation of GoToPlace
* Adding Planner::setup()
* Add a publisher wrapper and tweak the Task API
* Update header
* Get namespaces up to date
* fix compile error
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* Small tweaks
* uncomment mistakenly commented out code
* Merge branch 'feature/rxcpp' of ssh://github.com/osrf/rmf_core into feature/rxcpp
* initial implementation of door open phase
* Check the liveliness of the callback before triggering it
* More implementation
* pass back shared_ptr instead of raw pointer to ensure lifetime
* Flesh out more of the design
* Implement the new Task class
* Remove unnecessary enable_shared_from_this
* Sketching up the rmf_fleet_adapter::Task API
* Thinking about Task Phase API
* Have the plan start computation account for the map name
* Fix merge conflicts
* Fix merge conflicts (`#102 <https://github.com/osrf/rmf_core/issues/102>`_)
* Complete traffic negotiation system (`#101 <https://github.com/osrf/rmf_core/issues/101>`_)
* Merge in latest changes
* Fix negotiation bugs for read_only adapter
* Fix implementation bugs (`#100 <https://github.com/osrf/rmf_core/issues/100>`_)
* Finished updating fleet adapters -- need to track down bad optional access
* Appending new task to task_queue (`#99 <https://github.com/osrf/rmf_core/issues/99>`_)
* Merge branch 'merge/lenient_vicinity' into test/edge_cases
* Fix style
* Merging in lenient vicinity changes
* some packages dependencies fixes (`#94 <https://github.com/osrf/rmf_core/issues/94>`_)
* RMF Linter (`#96 <https://github.com/osrf/rmf_core/issues/96>`_)
* Improve fleet adapter (`#92 <https://github.com/osrf/rmf_core/issues/92>`_)
* multiple deliveries (`#88 <https://github.com/osrf/rmf_core/issues/88>`_)
  * implement multiple deliveries
  * refactor to be consistent with rmf_core coding format
* Migrate fleet adapter to use the schedule participant system (`#84 <https://github.com/osrf/rmf_core/issues/84>`_)
* Updated dependencies (`#80 <https://github.com/osrf/rmf_core/issues/80>`_)
* cleaned up deprecated documentation and examples (`#72 <https://github.com/osrf/rmf_core/issues/72>`_)
* fixed compilation errors in fleet adapter using new compute-start utility function (`#69 <https://github.com/osrf/rmf_core/issues/69>`_)
* Feature/refactor merge to graph (`#67 <https://github.com/osrf/rmf_core/issues/67>`_)
  * shifted api and implementation into rmf_traffic::agv::Planner, no longer rclcpp dependent, builds
  * problems returning from the optional StartSet
  * figured out how to use value of optional
  * tests should be quite complete, starting PR and cleanup
  * getting conflicted lane indices in tests
  * finished writing tests and passes
  * removed redundant planners in tests, fixed typos, added const to references
* Bring back all actions in the delivery task (`#66 <https://github.com/osrf/rmf_core/issues/66>`_)
* Changes that were used for DP2 (`#64 <https://github.com/osrf/rmf_core/issues/64>`_)
* Make the schedule manager persistent (`#63 <https://github.com/osrf/rmf_core/issues/63>`_)
* require the lift name parameter to be present for the mock lifts
* Use the correct string
* Fix bug
* Ignore lift times to avoid clock sync problems (`#62 <https://github.com/osrf/rmf_core/issues/62>`_)
* Allow the schedule to jump forward (`#61 <https://github.com/osrf/rmf_core/issues/61>`_)
* Task aggregator (`#55 <https://github.com/osrf/rmf_core/issues/55>`_)
* Tweaks (`#60 <https://github.com/osrf/rmf_core/issues/60>`_)
* Fix deliveries (`#59 <https://github.com/osrf/rmf_core/issues/59>`_)
* Fixing event listeners (`#58 <https://github.com/osrf/rmf_core/issues/58>`_)
* Be smarter about task IDs (`#57 <https://github.com/osrf/rmf_core/issues/57>`_)
* Add a docking event to the graph and planner (`#56 <https://github.com/osrf/rmf_core/issues/56>`_)
* Fixes (`#54 <https://github.com/osrf/rmf_core/issues/54>`_)
* Fleet adapter behavior fixes (`#51 <https://github.com/osrf/rmf_core/issues/51>`_)
* Create nodes to supervise the lift and door requests (`#49 <https://github.com/osrf/rmf_core/issues/49>`_)
* Make a main() function for the full_control fleet adapter (`#48 <https://github.com/osrf/rmf_core/issues/48>`_)
* Create a task for looping between two points an arbitrary number of times (`#47 <https://github.com/osrf/rmf_core/issues/47>`_)
* Implement a responsive full-control fleet adapter (`#46 <https://github.com/osrf/rmf_core/issues/46>`_)
* Read Only Fleet Adapter Tests (`#38 <https://github.com/osrf/rmf_core/issues/38>`_)
* Add replace and delay services to the schedule (`#32 <https://github.com/osrf/rmf_core/issues/32>`_)
* Read only fleet adapter (`#29 <https://github.com/osrf/rmf_core/issues/29>`_)
* fixed clean build issue (`#28 <https://github.com/osrf/rmf_core/issues/28>`_)
* some docs (`#25 <https://github.com/osrf/rmf_core/issues/25>`_)
  * some docs
  * english bad
* Integrate read-only fleet adapter (`#18 <https://github.com/osrf/rmf_core/issues/18>`_)
* Contributors: Aaron Chong, Charayaphan Nakorn Boon Han, Grey, Marco A. Gutiérrez, Michael X. Grey, Yadu, Yadunund, koonpeng, methylDragon
