^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_battery
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2021-XX-XX)
------------------
* Provides core `rmf_battery::agv` utilities
    * `BatterySystem` - Describe battery properties of the vehicle
    * `MechanicalSystem` - Describe mechanical properties of the vehicle
    * `PowerSystem` - Describe a power system on-board the vehicle
* Provides `rmf_battery::agv::SimpleMotionPowerSink`
    * Sample implementation of `rmf_battery::MotionPowerSink`
    * Compute change in battery charge due to locomotion over a specified `rmf_traffic::Trajectory`
* Provides `rmf_traffic::agv::SimpleDevicePowerSink`
    * Sample implementation of `rmf_battery::DevicePowerSink`
    * Compute change in battery charge due to power systems over a specified duration
* Contributors: Yadunund
