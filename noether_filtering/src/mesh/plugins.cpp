#include <pluginlib/class_list_macros.h>

// Mesh filters
#include "noether_filtering/mesh/bspline_reconstruction.h"

PLUGINLIB_EXPORT_CLASS(noether_filtering::mesh::BSplineReconstruction, noether_filtering::mesh::MeshFilterBase)
