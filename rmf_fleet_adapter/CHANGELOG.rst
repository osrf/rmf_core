^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_fleet_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.1 (2020-07-20)
------------------
* Interrupt dangling negotiation planning efforts to reduce memory usage: [#130](https://github.com/osrf/rmf_core/pull/130/)
* Trim the amount of system memory that is committed to a fleet adapter after each task: [#130](https://github.com/osrf/rmf_core/pull/130/)

1.0.0 (2020-06-23)
------------------
* Provides `rmf_fleet_adapter` library
    * The `rmf_fleet_adapter::agv` component can be used to develop a custom "Full Control" fleet adapter
    * `rmf_fleet_adapter/StandardNames.hpp` specifies topic names that are used for RMF integration
* Provides a prototype `read_only` fleet adapter implementation
    * This will be deprecated in the future in favor of a C++ API
    * To use this fleet adapter, you must implement a "read-only fleet driver" to talk to the fleet adapter using `rmf_fleet_msgs`
* Provides a deprecated `full_control` fleet adapter implementation
    * This is made to be backwards compatible with "full-control fleet drivers" that were developed in the early stages of RMF
    * New users should prefer to implement their own fleet adapter using the `rmf_fleet_adapter::agv` API
* Uses rxcpp to make the fleet adapters reactive and multi-threaded
* Has a known memory leak issue which will be resolved in a later release
* Contributors: Aaron Chong, Charayaphan Nakorn Boon Han, Marco A. Gutiérrez, Grey, Yadu, Yadunund, koonpeng, methylDragon
