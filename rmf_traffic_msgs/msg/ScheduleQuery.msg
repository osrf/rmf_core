
uint8 SPACETIME=1
uint8 TIMESPAN=2

uint8 type

# =====================
# ===== SPACETIME =====
# If SPACETIME mode is chosen, this will contain the regions to query
Region[] regions

# If SPACETIME mode is chosen, this will contain the box descriptions
Box[] boxes

# If SPACETIME mode is chosen, this will contain the circle descriptions
Circle[] circles

# If SPACETIME mode is chosen, this will contain the simple polygon descriptions
# TODO(MXG): Implement SimplePolygon
# SimplePolygon[] simple_polygons

# =====================
# ===== TIMESPAN ======
Timespan timespan
