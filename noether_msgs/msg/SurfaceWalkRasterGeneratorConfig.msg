float64 point_spacing 		        # Required spacing between path points
float64 raster_spacing 			# Offset between two rasters
float64 tool_offset 			# How far off the surface the tool needs to be
float64 intersection_plane_height 	# Used by the raster_tool_path_planner when offsetting to an adjacent path, a new plane has to be formed, but not too big
float64 min_hole_size 			# A path may pass over holes smaller than this, but must be broken when larger holes are encounterd.
float64 min_segment_size 		# The minimum segment size to allow when finding intersections; small segments will be discarded
float64 raster_rot_offset               # Specifies the direction of the raster path in radians
bool generate_extra_rasters   # Whether an extra raster stroke should be added to the beginning and end of the raster pattern
geometry_msgs/Vector3 cut_direction  # Desired raster direction, in mesh coordinate system (i.e. global coordinate system)
