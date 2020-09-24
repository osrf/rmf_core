^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_task_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2020-09-24)
------------------
* Implement Ingestors (`#164 <https://github.com/osrf/rmf_core/issues/164>`_)
  * Implement Ingestors
  Split the existing Dispensers into Dispensers and Ingestors. Ingestors
  will handle the taking in of objects delivered to them.
  Added a new set of messages for the Ingestors, which mimics the existing
  Dispenser messages.
  Modified the Delivery task to make use of the Ingestors.
  Added unit tests for the Ingestor and modified the existing Delivery test
  to use a flaky Ingestor instead of a flaky Dispenser.
  * Modify maintainer name for rmf_ingestor_msgs package
* Traffic Light API (`#147 <https://github.com/osrf/rmf_core/issues/147>`_)
  * Introduce the Traffic Light API
  * Bump to version 1.1.0
* Contributors: Grey, Rushyendra Maganty

1.0.0 (2020-06-23)
------------------
* Initial set of messages for describing task requests and results over ROS2
* Contributors: Grey, Marco A. Gutiérrez, Michael X. Grey, Morgan Quigley
