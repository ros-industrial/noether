#include <boost/preprocessor/seq/for_each.hpp>
#include <class_loader/register_macro.hpp>
#include "noether_filtering/cloud/voxel_grid_filter.h"
#include "noether_filtering/cloud/statistical_outlier_filter.h"
#include "noether_filtering/cloud/crop_box_filter.h"
#include "noether_filtering/cloud/pass_through_filter.h"
#include "noether_filtering/mesh/bspline_reconstruction.h"
#include <pcl/point_types.h>

#define CREATE_FILTER_PLUGIN_IMPL(r, FILTER_TYPE, POINT_TYPE) \
  CLASS_LOADER_REGISTER_CLASS(FILTER_TYPE<POINT_TYPE>, noether_filtering::FilterBase<pcl::PointCloud<POINT_TYPE>::Ptr>)

#define CREATE_FILTER_PLUGINS(FILTER_TYPE, POINT_TYPES) \
  BOOST_PP_SEQ_FOR_EACH(CREATE_FILTER_PLUGIN_IMPL, FILTER_TYPE, POINT_TYPES)

namespace noether_filtering
{
  // Point Cloud Filters
  CREATE_FILTER_PLUGINS(VoxelGridFilter, PCL_XYZ_POINT_TYPES)
  CREATE_FILTER_PLUGINS(StatisticalOutlierFilter, PCL_XYZ_POINT_TYPES)
  CREATE_FILTER_PLUGINS(CropBoxFilter, PCL_XYZ_POINT_TYPES)
  CREATE_FILTER_PLUGINS(PassThroughFilter, PCL_XYZ_POINT_TYPES)

  // Mesh Filters
  CLASS_LOADER_REGISTER_CLASS(BSplineReconstruction, FilterBase<pcl::PolygonMesh>);
}
