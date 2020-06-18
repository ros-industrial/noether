#include <pluginlib/class_list_macros.h>

// Mesh filters
#include "noether_filtering/mesh/euclidean_clustering.h"
#include "noether_filtering/mesh/clean_data.h"
#include "noether_filtering/mesh/windowed_sinc_smoothing.h"
#include "noether_filtering/mesh/fill_holes.h"

PLUGINLIB_EXPORT_CLASS(noether_filtering::mesh::EuclideanClustering, noether_filtering::mesh::MeshFilterBase)
PLUGINLIB_EXPORT_CLASS(noether_filtering::mesh::CleanData, noether_filtering::mesh::MeshFilterBase)
PLUGINLIB_EXPORT_CLASS(noether_filtering::mesh::WindowedSincSmoothing, noether_filtering::mesh::MeshFilterBase)
PLUGINLIB_EXPORT_CLASS(noether_filtering::mesh::FillHoles, noether_filtering::mesh::MeshFilterBase)

#ifdef ENABLE_NURBS
#include "noether_filtering/mesh/bspline_reconstruction.h"
PLUGINLIB_EXPORT_CLASS(noether_filtering::mesh::BSplineReconstruction, noether_filtering::mesh::MeshFilterBase)
#endif
