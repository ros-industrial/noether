float64 raster_spacing 	    # Offset between two rasters
float64 point_spacing 	     # Required spacing between path points
float64 raster_rot_offset    # Specifies the direction of the raster path in radians. Rotates about short dimension of bounding box.
float64 min_segment_size     # The minimum segment size to allow when finding intersections; small segments will be discarded
float64 min_hole_size 	     # A path may pass over holes smaller than this, but must be broken when larger holes are encounterd.
float64 tool_offset 	     # How far off the surface the tool needs to be
bool generate_extra_rasters  # Whether an extra raster stroke should be added to the beginning and end of the raster pattern

# When set to TRUE: initial cuts are determined using an axis-aligned bounding box. (global axes)
# When set to FALSE: initial cuts are determined using an object-oriented bounding box and the
# principal axes of the part.  (Note that ROS sets message fields to 0/False by default.)
bool raster_wrt_global_axes

# Normal of the cutting planes.  Points from the first raster to the second raster, not the
# direction from the first point in one raster to the second point of the same raster.  Specified
# in the frame of the input mesh.  Overrides raster_wrt_global_axes
geometry_msgs/Vector3 raster_direction
