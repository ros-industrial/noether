float64 point_spacing 		      # Required spacing between path points
float64 raster_spacing 		      # Offset between two rasters
float64 tool_offset 		      # How far off the surface the tool needs to be
float64 min_hole_size 		      # A path passes over smaller holes breaks into two on larger wholes 
float64 min_segment_size 	      # The minimum segment size to allowed 
float64 raster_rot_offset             # use this angle when no sources are povided
bool generate_extra_rasters           # Add extra strokes off the boundaries (NOT YET IMPLEMENTED BY HEAT METHOD)