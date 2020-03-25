
# A 2D shape reference.

uint8 NONE=0
uint8 BOX=1
uint8 CIRCLE=2

# TODO(MXG): Add support for the SimplePolygon class
# uint8 SIMPLE_POLYGON=3

# Choose between the BOX and CIRCLE types
uint8 type

# Specify the index of the shape. We support 256 different convex shapes per
# trajectory. If more shapes are needed than that, then the trajectory must be
# split into more trajectories.
uint8 index
