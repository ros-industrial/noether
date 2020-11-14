#include <boost/preprocessor/seq/for_each.hpp>
#include <pluginlib/class_list_macros.h>
#include <pcl/point_types.h>

// Cloud Filters
#include "noether_filtering/cloud/voxel_grid_filter.h"
#include "noether_filtering/cloud/statistical_outlier_filter.h"
#include "noether_filtering/cloud/crop_box_filter.h"
#include "noether_filtering/cloud/pass_through_filter.h"
#include "noether_filtering/cloud/radius_outlier_filter.h"
#include "noether_filtering/cloud/mls_smoothing_filter.h"

#define CREATE_FILTER_PLUGIN_IMPL(r, FILTER_TYPE, POINT_TYPE)                                                          \
  PLUGINLIB_EXPORT_CLASS(FILTER_TYPE<POINT_TYPE>, noether_filtering::FilterBase<pcl::PointCloud<POINT_TYPE>>)

#define CREATE_FILTER_PLUGINS(FILTER_TYPE, POINT_TYPES)                                                                \
  BOOST_PP_SEQ_FOR_EACH(CREATE_FILTER_PLUGIN_IMPL, FILTER_TYPE, POINT_TYPES)

// Point Cloud Filters
CREATE_FILTER_PLUGINS(noether_filtering::cloud::VoxelGridFilter, PCL_XYZ_POINT_TYPES)
CREATE_FILTER_PLUGINS(noether_filtering::cloud::StatisticalOutlierFilter, PCL_XYZ_POINT_TYPES)
CREATE_FILTER_PLUGINS(noether_filtering::cloud::CropBoxFilter, PCL_XYZ_POINT_TYPES)
CREATE_FILTER_PLUGINS(noether_filtering::cloud::PassThroughFilter, PCL_XYZ_POINT_TYPES)
CREATE_FILTER_PLUGINS(noether_filtering::cloud::RadiusOutlierFilter, PCL_XYZ_POINT_TYPES)
CREATE_FILTER_PLUGINS(noether_filtering::cloud::MLSSmoothingFilter, PCL_XYZ_POINT_TYPES)
