# Resolution for the octree
float64 octree_res=0.005

# Radious used for grouping
float64 search_radius=0.01

# Desired omp thread count
uint64 num_threads=4

# Points with "v > max_v * (1 - neighbor_tol)" are considered edge points
float64 neighbor_tol=0.01

# Used in downsampling edge points
float64 voxel_size=0.01

# Downsamples only when edge cluster size is greater than this value
int32 edge_cluster_min=3

# Precision used for nearest neighbor searches
float64 kdtree_epsilon=0.01

# Minimum distance by which to project an edge point along the normal, recommended 2 * octree_res
float64 min_projection_dist=0.01

# Will split when distance exceeds octree_res * max_intersecting_voxels
int32 max_intersecting_voxels=4

# Any two consecutive points with a shortest distance smaller than this value are merged
float64 merge_dist=0.01
