#include <boost/make_shared.hpp>
#include <gtest/gtest.h>
#include "noether_filtering/filter_manager.h"
#include "noether_filtering/utils.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "noether_filtering/cloud/crop_box_filter.h"
#include "noether_filtering/cloud/pass_through_filter.h"
#include "noether_filtering/cloud/radius_outlier_filter.h"
#include "noether_filtering/cloud/statistical_outlier_filter.h"
#include "noether_filtering/cloud/voxel_grid_filter.h"
#include "noether_filtering/cloud/mls_smoothing_filter.h"

template <typename PointT>
XmlRpc::XmlRpcValue createVoxelGridConfig()
{
  using namespace noether_filtering;
  using namespace noether_filtering::cloud;
  using namespace noether_filtering::config_fields::filter;
  XmlRpc::XmlRpcValue f;
  f[NAME] = "voxel_grid";
  f[TYPE_NAME] = utils::getClassName<VoxelGridFilter<PointT>>();

  XmlRpc::XmlRpcValue vg;
  vg[VoxelGridFilter<PointT>::LEAF_SIZE] = 0.1;
  f[CONFIG] = std::move(vg);

  return f;
}

template <typename PointT>
XmlRpc::XmlRpcValue createStatisticalOutlierConfig()
{
  using namespace noether_filtering;
  using namespace noether_filtering::cloud;
  using namespace noether_filtering::config_fields::filter;
  XmlRpc::XmlRpcValue f;
  f[NAME] = "statistical_outlier";
  f[TYPE_NAME] = utils::getClassName<StatisticalOutlierFilter<PointT>>();

  XmlRpc::XmlRpcValue so;
  so[StatisticalOutlierFilter<PointT>::MEAN_K] = 10;
  so[StatisticalOutlierFilter<PointT>::STD_DEV_MULT] = 1.0;
  f[CONFIG] = std::move(so);

  return f;
}

template <typename PointT>
XmlRpc::XmlRpcValue createCropBoxConfig()
{
  using namespace noether_filtering;
  using namespace noether_filtering::cloud;
  using namespace noether_filtering::config_fields::filter;
  XmlRpc::XmlRpcValue f;
  f[NAME] = "crop_box";
  f[TYPE_NAME] = utils::getClassName<CropBoxFilter<PointT>>();

  XmlRpc::XmlRpcValue min;
  min["x"] = -1.0;
  min["y"] = -1.0;
  min["z"] = -1.0;

  XmlRpc::XmlRpcValue max;
  max["x"] = 1.0;
  max["y"] = 1.0;
  max["z"] = 1.0;

  XmlRpc::XmlRpcValue t;
  t["x"] = 0.5;
  t["y"] = 0.5;
  t["z"] = 0.5;
  t["rx"] = 0.1;
  t["ry"] = 0.1;
  t["rz"] = 0.1;

  XmlRpc::XmlRpcValue cb;
  cb[CropBoxFilter<PointT>::MIN] = min;
  cb[CropBoxFilter<PointT>::MAX] = max;
  cb[CropBoxFilter<PointT>::TRANSFORM] = t;
  cb[CropBoxFilter<PointT>::CROP_OUTSIDE] = false;

  f[CONFIG] = std::move(cb);

  return f;
}

template <typename PointT>
XmlRpc::XmlRpcValue createPassThroughConfig()
{
  using namespace noether_filtering;
  using namespace noether_filtering::cloud;
  using namespace noether_filtering::config_fields::filter;
  XmlRpc::XmlRpcValue f;
  f[NAME] = "pass_through";
  f[TYPE_NAME] = utils::getClassName<PassThroughFilter<PointT>>();

  XmlRpc::XmlRpcValue pt;
  pt[PassThroughFilter<PointT>::FILTER_FIELD_NAME] = "y";
  pt[PassThroughFilter<PointT>::MIN_LIMIT] = -1.0;
  pt[PassThroughFilter<PointT>::MAX_LIMIT] = 1.0;
  pt[PassThroughFilter<PointT>::NEGATIVE] = false;
  f[CONFIG] = std::move(pt);

  return f;
}

template <typename PointT>
XmlRpc::XmlRpcValue createRadiusOutlierConfig()
{
  using namespace noether_filtering;
  using namespace noether_filtering::cloud;
  using namespace noether_filtering::config_fields::filter;
  XmlRpc::XmlRpcValue f;
  f[NAME] = "radius_outlier_filter";
  f[TYPE_NAME] = utils::getClassName<RadiusOutlierFilter<PointT>>();

  XmlRpc::XmlRpcValue ro;
  ro[RadiusOutlierFilter<PointT>::RADIUS] = 1.0;
  ro[RadiusOutlierFilter<PointT>::MIN_PTS] = 5;
  f[CONFIG] = std::move(ro);

  return f;
}

template <typename PointT>
XmlRpc::XmlRpcValue createMLSSmoothingConfig()
{
  using namespace noether_filtering;
  using namespace noether_filtering::cloud;
  using namespace noether_filtering::config_fields::filter;
  XmlRpc::XmlRpcValue f;
  f[NAME] = "mls_smoothing_filter";
  f[TYPE_NAME] = utils::getClassName<MLSSmoothingFilter<PointT>>();

  XmlRpc::XmlRpcValue mls;
  mls[MLSSmoothingFilter<PointT>::SEARCH_RADIUS] = 0.1;
  mls[MLSSmoothingFilter<PointT>::POLYNOMIAL_ORDER] = 2;
  f[CONFIG] = std::move(mls);

  return f;
}

template <typename PointT>
XmlRpc::XmlRpcValue createManagerConfig(std::string group_name)
{
  using namespace noether_filtering::config_fields;

  // Create filter config(s)
  XmlRpc::XmlRpcValue filters;
  filters.setSize(6);
  filters[0] = createVoxelGridConfig<PointT>();
  filters[1] = createStatisticalOutlierConfig<PointT>();
  filters[2] = createCropBoxConfig<PointT>();
  filters[3] = createPassThroughConfig<PointT>();
  filters[4] = createRadiusOutlierConfig<PointT>();
  filters[5] = createMLSSmoothingConfig<PointT>();

  // Create a single filter group
  XmlRpc::XmlRpcValue g;
  g[group::GROUP_NAME] = std::move(group_name);
  g[group::CONTINUE_ON_FAILURE] = false;
  g[group::FILTERS] = std::move(filters);

  // Create a collection of filter groups
  XmlRpc::XmlRpcValue groups;
  groups.setSize(1);
  groups[0] = std::move(g);

  // Create the manager configuration
  XmlRpc::XmlRpcValue manager;
  manager[manager::FILTER_GROUPS] = std::move(groups);

  return manager;
}

template <typename PointT>
class FilterManagerFixture : public testing::Test
{
public:
  using testing::Test::Test;
  std::shared_ptr<noether_filtering::FilterManager<pcl::PointCloud<PointT>>> manager;
};

typedef ::testing::Types<pcl::PointXYZ, pcl::PointXYZRGB, pcl::PointNormal, pcl::PointXYZI> Implementations;

TYPED_TEST_SUITE(FilterManagerFixture, Implementations);

TYPED_TEST(FilterManagerFixture, FilterManagerTest)
{
  using namespace noether_filtering;
  using namespace noether_filtering::config_fields::filter;
  using FilterT = noether_filtering::FilterBase<pcl::PointCloud<TypeParam>>;
  using Cloud = pcl::PointCloud<TypeParam>;

  const std::string group_name = "test_group";
  XmlRpc::XmlRpcValue config = createManagerConfig<TypeParam>(group_name);

  this->manager = std::make_shared<FilterManager<Cloud>>(utils::getClassName<FilterT>());
  ASSERT_TRUE(this->manager->init(config));

  using Group = FilterGroup<Cloud>;
  std::shared_ptr<Group> group = this->manager->getFilterGroup(group_name);

  ASSERT_TRUE(group != nullptr);

  pcl::PointCloud<TypeParam> input_cloud;
  input_cloud.points.resize(100);
  pcl::PointCloud<TypeParam> output_cloud;
  std::string error;

  ASSERT_TRUE(group->applyFilters(input_cloud, output_cloud, error));

  ASSERT_LE(output_cloud.points.size(), input_cloud.points.size());
}

int main(int argc, char** argv)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
