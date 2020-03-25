
# A 2D box or circle shape reference.

uint8 NONE=0
uint8 BOX=1
uint8 CIRCLE=2

# Choose between the BOX and CIRCLE types
uint8 type

# Specify the index of the shape. We support 256 different convex shapes per
# context.
uint8 index
