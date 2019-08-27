# Each lane segment consists of a straight line of traffic with a surrounding
# polygon of space allocated to it. Lanes can be unidirectional (most common)
# or bidirectional (if corridor space is too constrained for two lanes)

LanePoint start
LanePoint end

Polygon polygon

uint8 flow
uint8 FLOW_UNIDIRECTIONAL=0
uint8 FLOW_BIDIRECTIONAL=1
