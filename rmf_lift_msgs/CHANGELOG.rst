^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_lift_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-23)
------------------
* Fix merge conflicts
* Fix merge conflicts (`#102 <https://github.com/osrf/rmf_core/issues/102>`_)
* geometry_msgs is not needed, removing from cmake (`#93 <https://github.com/osrf/rmf_core/issues/93>`_)
  Removing the find_package pointing to geometry_msgs as it is not needed and fails when creating the package due to a dependency not found.
* add request_time to LiftRequest message (`#52 <https://github.com/osrf/rmf_core/issues/52>`_)
* lift messages (`#22 <https://github.com/osrf/rmf_core/issues/22>`_)
* Contributors: Grey, Marco A. Gutiérrez, Michael X. Grey, Morgan Quigley
