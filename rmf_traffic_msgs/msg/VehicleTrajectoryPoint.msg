# The meaning of 't' will depend on the context in which this Point is used.
# For path proposals, 't' indicates seconds after the start of the segment,
# since planar path segments are linked by vertical lift operations, which are
# not possible for the proposer to estimate with precision.
float32 t

# (x, y) units are meters. +x = east, +y = north
float32 x
float32 y

# yaw units are radians, 0 = +x (east), anti-clockwise
float32 yaw
