int32 min_cluster_size 				# The minimum number of cells (triangles) allowed in a segment
int32 max_cluster_size  				# The maximum number of cells (triangles) allowed in a segment (currently unimplemented 1/10/19)
float64 curvature_threshold                     # The desired threshold for "nearness" in radians
float64 neighborhood_radius         # Radius used by MLS to calculate point normals.
bool use_mesh_normals             # If true, segmentation is based off of the normals defined by each triangle vertex via the right hand rule. If false, normals are calculated using neighborhood size

