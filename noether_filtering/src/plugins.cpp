#include <boost/preprocessor/seq/for_each.hpp>
#include <class_loader/register_macro.hpp>
#include <pcl/point_types.h>

// Cloud Filters
#include "noether_filtering/cloud/voxel_grid_filter.h"
#include "noether_filtering/cloud/statistical_outlier_filter.h"
#include "noether_filtering/cloud/crop_box_filter.h"
#include "noether_filtering/cloud/pass_through_filter.h"
#include "noether_filtering/cloud/radius_outlier_filter.h"

// Mesh filters
#include "noether_filtering/mesh/bspline_reconstruction.h"

#define CREATE_FILTER_PLUGIN_IMPL(r, FILTER_TYPE, POINT_TYPE) \
  CLASS_LOADER_REGISTER_CLASS(FILTER_TYPE<POINT_TYPE>, noether_filtering::FilterBase<pcl::PointCloud<POINT_TYPE>::Ptr>)

#define CREATE_FILTER_PLUGINS(FILTER_TYPE, POINT_TYPES) \
  BOOST_PP_SEQ_FOR_EACH(CREATE_FILTER_PLUGIN_IMPL, FILTER_TYPE, POINT_TYPES)

// Point Cloud Filters
CREATE_FILTER_PLUGINS(noether_filtering::VoxelGridFilter, PCL_XYZ_POINT_TYPES)
CREATE_FILTER_PLUGINS(noether_filtering::StatisticalOutlierFilter, PCL_XYZ_POINT_TYPES)
CREATE_FILTER_PLUGINS(noether_filtering::CropBoxFilter, PCL_XYZ_POINT_TYPES)
CREATE_FILTER_PLUGINS(noether_filtering::PassThroughFilter, PCL_XYZ_POINT_TYPES)
CREATE_FILTER_PLUGINS(noether_filtering::RadiusOutlierFilter, PCL_XYZ_POINT_TYPES)

// Mesh Filters
CLASS_LOADER_REGISTER_CLASS(noether_filtering::BSplineReconstruction, noether_filtering::FilterBase<pcl::PolygonMesh>);
