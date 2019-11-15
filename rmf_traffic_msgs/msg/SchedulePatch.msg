
ScheduleChangeInsert[] insertions

ScheduleChangeInterrupt[] interruptions

ScheduleChangeDelay[] delays

ScheduleChangeReplace[] replacements

ScheduleChangeErase[] erasures

# TODO(MXG): The database will only ever report 1 cull per update. Consider
# making this a single field instead of a dynamic array.
ScheduleChangeCull[] culls

uint64 latest_version
