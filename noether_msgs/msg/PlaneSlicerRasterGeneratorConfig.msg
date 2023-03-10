float64 raster_spacing 	   # Offset between two rasters
float64 point_spacing 	   # Required spacing between path points
float64 raster_rot_offset  # Specifies the direction of the raster path in radians
float64 min_segment_size   # The minimum segment size to allow when finding intersections; small segments will be discarded
float64 min_hole_size 	   # A path may pass over holes smaller than this, but must be broken when larger holes are encounterd.
float64 tool_offset 	   # How far off the surface the tool needs to be
float64 search_radius 	   # Radius to average normals, set equal or lower to zero to not average normals.
