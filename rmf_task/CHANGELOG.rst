^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_task
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2021-XX-XX)
------------------
* Provides `rmf_task::requests` to describe various task requests which may be used for allocation planning
* Provides `rmf_task::agv::TaskPlanner` object that can generate task allocation plans where a set of `rmf_task::requests` are optimally distributed across a set of agents
    * `rmf_task::requests::ChargeBattery` requests are automatically injected into the allocation set where necessary
