
# Name of the participant
string name

# Owner of the participant (e.g. fleet name)
string owner

uint8 RX_INVALID = 0
uint8 RX_UNRESPONSIVE = 1
uint8 RX_RESPONSIVE = 2

# Whether or not this participant will automatically respond to conflicts
uint8 responsiveness

# The physical profile of this Participant
Profile profile
