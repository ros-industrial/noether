#include <boost/make_shared.hpp>
#include <gtest/gtest.h>
#include "noether_filtering/filter_group.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<typename PointT>
XmlRpc::XmlRpcValue createVoxelGridConfig(std::string pt_name)
{
  using namespace noether_filtering::config_field_names;
  XmlRpc::XmlRpcValue f;
  f[NAME] = "voxel_grid";
  f[TYPE_NAME] = "VoxelGridFilter<" + std::move(pt_name) + ">";

  XmlRpc::XmlRpcValue vg;
  vg["leaf_size"] = 0.1;
  f[CONFIG] = std::move(vg);

  return f;
}

template<typename PointT>
XmlRpc::XmlRpcValue createStatisticalOutlierConfig(std::string pt_name)
{
  using namespace noether_filtering::config_field_names;
  XmlRpc::XmlRpcValue f;
  f[NAME] = "statistical_outlier";
  f[TYPE_NAME] = "StatisticalOutlierFilter<" + std::move(pt_name) + ">";

  XmlRpc::XmlRpcValue so;
  so["mean_k"] = 10;
  so["std_dev_mult"] = 1.0;
  f[CONFIG] = std::move(so);

  return f;
}

template<typename PointT>
XmlRpc::XmlRpcValue createCropBoxConfig(std::string pt_name)
{
  using namespace noether_filtering::config_field_names;
  XmlRpc::XmlRpcValue f;
  f[NAME] = "crop_box";
  f[TYPE_NAME] = "CropBoxFilter<" + std::move(pt_name) + ">";

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
  cb["min"] = min;
  cb["max"] = max;
  cb["transform"] = t;
  cb["crop_outside"] = false;

  f[CONFIG] = std::move(cb);

  return f;
}

template<typename PointT>
XmlRpc::XmlRpcValue createPassThroughConfig(std::string pt_name)
{
  using namespace noether_filtering::config_field_names;
  XmlRpc::XmlRpcValue f;
  f[NAME] = "pass_through";
  f[TYPE_NAME] = "PassThroughFilter<" + std::move(pt_name) + ">";

  XmlRpc::XmlRpcValue pt;
  pt["filter_field_name"] = "y";
  pt["min_limit"] = -1.0;
  pt["max_limit"] = 1.0;
  pt["negative"] = false;
  f[CONFIG] = std::move(pt);

  return f;
}

template<typename PointT>
XmlRpc::XmlRpcValue createRadiusOutlierConfig(std::string pt_name)
{
  using namespace noether_filtering::config_field_names;
  XmlRpc::XmlRpcValue f;
  f[NAME] = "radius_outlier_filter";
  f[TYPE_NAME] = "RadiusOutlierFilter<" + std::move(pt_name) + ">";

  XmlRpc::XmlRpcValue ro;
  ro["radius"] = 1.0;
  ro["min_pts"] = 5;
  f[CONFIG] = std::move(ro);

  return f;
}

template<typename T>
class FilterManagerFixture : public testing::Test
{
public:
  using testing::Test::Test;

  noether_filtering::FilterGroup<typename pcl::PointCloud<T>::Ptr> manager;
};

typedef ::testing::Types<pcl::PointXYZ, pcl::PointXYZRGB, pcl::PointNormal> Implementations;

TYPED_TEST_CASE(FilterManagerFixture, Implementations);

TYPED_TEST(FilterManagerFixture, FilterManagerTest)
{
  using namespace noether_filtering::config_field_names;

  XmlRpc::XmlRpcValue config;
  config[CONTINUE_ON_FAILURE] = true;

  XmlRpc::XmlRpcValue filters;
  filters.setSize(5);

  // Get the name of the current test type (i.e. the name of the PCL point type)
  auto info = ::testing::UnitTest::GetInstance()->current_test_info();
  std::string type_param(info->type_param());

  // Create filter config(s)
  filters[0] = createVoxelGridConfig<TypeParam>(type_param);
  filters[1] = createStatisticalOutlierConfig<TypeParam>(type_param);
  filters[2] = createCropBoxConfig<TypeParam>(type_param);
  filters[3] = createPassThroughConfig<TypeParam>(type_param);
  filters[4] = createRadiusOutlierConfig<TypeParam>(type_param);
  config[FILTERS] = filters;

  ASSERT_TRUE(this->manager.init(config));

  typename pcl::PointCloud<TypeParam>::Ptr input_cloud
    = boost::make_shared<pcl::PointCloud<TypeParam>>();
  input_cloud->points.resize(100);
  typename pcl::PointCloud<TypeParam>::Ptr output_cloud;
  std::string error;

  ASSERT_TRUE(this->manager.applyFilters(input_cloud, output_cloud, error));

  ASSERT_LE(output_cloud->points.size(), input_cloud->points.size());
}

int main(int argc, char **argv)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
