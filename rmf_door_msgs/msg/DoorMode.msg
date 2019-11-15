# The DoorMode message captures the "mode" of an automatic door controller.
# Most door controllers default to running in "closed" mode, and transition
# through some sort of "moving" mode until reaching the "open" mode.

uint32 mode

# door_mode must be one of the following enumerated values:
uint32 MODE_CLOSED=0
uint32 MODE_MOVING=1
uint32 MODE_OPEN=2
