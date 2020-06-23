^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_dispenser_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-23)
------------------
* Refactor delivery message (`#45 <https://github.com/osrf/rmf_core/issues/45>`_)
* reverting dispenser request item message to be used for fiducial tag name in CSSD workcell (`#44 <https://github.com/osrf/rmf_core/issues/44>`_)
* Workcell msgs (`#42 <https://github.com/osrf/rmf_core/issues/42>`_)
  * added some comments on DispenserResult, added workcell messages
  * added some more comments, common message fields for all workcell-type state messages as well
  * made it more obvious the uses of common fields in workcell-type state, request and result messages, added unused template messages
  * accidentally removed seconds-left field from dispenser state
* Change dispenser_name back to target_guid (`#40 <https://github.com/osrf/rmf_core/issues/40>`_)
* Change target_guid to dispenser_name (`#36 <https://github.com/osrf/rmf_core/issues/36>`_)
* simplifying DispenserRequest (`#35 <https://github.com/osrf/rmf_core/issues/35>`_)
* dispenser messages (`#23 <https://github.com/osrf/rmf_core/issues/23>`_)
* Contributors: Aaron Chong, Grey, Morgan Quigley
