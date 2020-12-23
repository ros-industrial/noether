# Resolution for the octree
float64 octree_res

# Radious used for grouping
float64 search_radius

# Desired omp thread count
uint64 num_threads

# Points with "v > max_v * (1 - neighbor_tol)" are considered edge points
float64 neighbor_tol

# Used in downsampling edge points
float64 voxel_size

# Downsamples only when edge cluster size is greater than this value
int32 edge_cluster_min

# Precision used for nearest neighbor searches
float64 kdtree_epsilon

# Minimum distance by which to project an edge point along the normal, recommended 2 * octree_res
float64 min_projection_dist

# Will split when distance exceeds octree_res * max_intersecting_voxels
int32 max_intersecting_voxels

# Any two consecutive points with a shortest distance smaller than this value are merged
float64 merge_dist

# maximum segment length
float64 max_segment_length
