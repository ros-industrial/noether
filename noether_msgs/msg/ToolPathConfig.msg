float64 pt_spacing 					# Required spacing between path points
float64 line_spacing 				# Offset between two rasters
float64 tool_offset 				# How far off the surface the tool needs to be
float64 intersecting_plane_height 	# Used by the raster_tool_path_planner when offsetting to an adjacent path, a new plane has to be formed, but not too big
float64 min_hole_size 				# A path may pass over holes smaller than this, but must be broken when larger holes are encounterd. 
float64 min_segment_size 			# The minimum segment size to allow when finding intersections; small segments will be discarded
float64 raster_angle          # Specifies the direction of the raster path in radians
bool raster_wrt_principal_axis # When true, 0 raster_angle is aligned with the principal axis
