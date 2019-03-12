float64 pt_spacing 					# Required spacing between path points
float64 line_spacing 				# Offset between two rasters
float64 tool_offset 				# How far off the surface the tool needs to be
float64 intersecting_plane_height 	# Used by the raster_tool_path_planner when offsetting to an adjacent path, a new plane has to be formed, but not too big
float64 min_hole_size 				# A path may pass over holes smaller than this, but must be broken when larger holes are encounterd. 
float64 min_segment_size 			# The minimum segment size to allow when finding intersections; small segments will be discarded
int32 raster_direction                          # Specifies the direction of the raster path wrt to the principal axes

# Options for raster_direction
int32 MAJOR_AXIS = 0                            # Along the largest axis of the mesh bounding box
int32 MIDDLE_AXIS = 1                           # Along the 2nd largest axis of the mesh bounding box
int32 MINOR_AXIS = 2                            # Along the smallest axis of the mesh bounding box
int32 MAJOR_MIDDLE_45 = 3                       # Halfway in between the major and middle axes
int32 MAJOR_MIDDLE_NEG45 = 4                    # Halfway in between major and negative middle axes
