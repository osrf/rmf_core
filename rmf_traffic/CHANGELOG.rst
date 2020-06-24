^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_traffic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-23)
------------------
* Provides core `rmf_traffic` utilities
    * `Trajectory` - Describe a motion through 2D space
    * `Route` - Describe a path that a robot will follow
    * `Motion` - Convert a discrete `Trajectory` into a continuous function
* Provides `rmf_traffic::schedule` utilities for managing traffic schedules
    * `Database` - Object for managing a schedule database
    * `Viewer` - Interface for viewing a schedule database
    * `Writer` - Interface for writing to a schedule database
    * `Mirror` - Object for mirroring a schedule database across a distributed system
    * `Snapshot` - Object that captures a snapshot of a database
    * `Participant` - Object that manages participation in a schedule
    * `ParticipantDescription` - Object that describes a participant
    * `Query` - Object that describes a schedule query
    * `Negotiation` - Object that manages a traffic negotiation
    * `Negotiator` - Interface used to respond to negotiation events
    * `StubbornNegotiator` - An implementation of a `Negotiator` that refuses to deviate from its path
* Provides `rmf_traffic::agv` utilities to help AGV fleets integrate with the schedule
    * `Graph` - Describe the route graph that an AGV is allowed to use
    * `VehicleTraits` - Describe the kinematic properties of an AGV
    * `Interpolate` - Interpolate the trajectory of an AGV based on its traits
    * `RouteValidator` - Interface for determining whether a route is free of conflicts
    * `Planner` - Object that can generate plans for an AGV that comply with the schedule or that suit a negotiation
    * `SimpleNegotiator` - An implementation of a `schedule::Negotiator` that can negotiate for an AGV
* Contributors: Aaron Chong, Boon Han, Charayaphan Nakorn Boon Han, Grey, Luca Della Vedova, Marco A. Gutiérrez, Morgan Quigley, Yadu, Yadunund, koonpeng
