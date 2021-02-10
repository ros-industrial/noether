uint8 POINT_SPACING_METHOD_NONE=0
uint8 POINT_SPACING_METHOD_MIN_DISTANCE=1
uint8 POINT_SPACING_METHOD_EQUAL_SPACING=2
uint8 POINT_SPACING_METHOD_PARAMETRIC_SPLINE=3

# Only edge segments with more than this many points will be returned
uint64 min_num_points

# True in order set the normal of each point as the average of the normal vectors of the points within a specified radius
bool normal_averaging

# The search radius used for normal averaging
float64 normal_search_radius

# A value [0, 1] that influences the normal averaged based on its distance, set to 0 to disable
float64 normal_influence_weight

# The method used for spacing the points, NONE = 0, MIN_DISTANCE = 1, EQUAL_SPACING = 2, PARAMETRIC_SPLINE = 3
int32 point_spacing_method

# Point distance parameter used in conjunction with the spacing method
float64 point_dist

# maximum segment length
float64 max_segment_length
