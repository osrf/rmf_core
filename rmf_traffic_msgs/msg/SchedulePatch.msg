
uint64[] unregister_participants

ScheduleRegister[] register_participants

# The changes to the schedule, grouped into the different participants
ScheduleParticipantPatch[] participants

# TODO(MXG): The database will only ever report 1 cull per update. Consider
# making this a single field instead of a dynamic array.
ScheduleChangeCull[] cull

uint64 latest_version
