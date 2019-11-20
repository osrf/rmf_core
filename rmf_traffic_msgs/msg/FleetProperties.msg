
# The identifier for the fleet that is submitting the trajectories
string fleet_id

# The different levels of control that RMF has over the target fleet, either
# full path and command controls, only stop/resume controls, or no control over
# it at all
uint8 TYPE_NO_CONTROL = 0
uint8 TYPE_RESPONSIVE = 1

# The level of control RMF have over the fleet
uint8 type
