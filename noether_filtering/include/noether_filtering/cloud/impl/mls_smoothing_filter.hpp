#ifndef NOETHER_FILTERING_CLOUD_IMPL_MLS_SMOOTHING_FILTER_HPP
#define NOETHER_FILTERING_CLOUD_IMPL_MLS_SMOOTHING_FILTER_HPP

#include "noether_filtering/cloud/mls_smoothing_filter.h"
#include "noether_filtering/utils.h"
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <console_bridge/console.h>
#include <XmlRpcException.h>

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
const std::string MLSSmoothingFilter<PointT>::POLYNOMIAL_ORDER = "polynomial_order";

template <typename PointT>
const std::string MLSSmoothingFilter<PointT>::SEARCH_RADIUS = "search_radius";

template <typename PointT>
bool MLSSmoothingFilter<PointT>::configure(XmlRpc::XmlRpcValue value)
{
  std::string error;
  if (!value.hasMember(SEARCH_RADIUS))
    error += SEARCH_RADIUS + ", ";
  if (!value.hasMember(POLYNOMIAL_ORDER))
    error += POLYNOMIAL_ORDER + ", ";

  if (!error.empty())
  {
    CONSOLE_BRIDGE_logError("Failed to find required configuration parameters: %s", error.c_str());
    return false;
  }

  try
  {
    params.search_radius = static_cast<double>(value[SEARCH_RADIUS]);
    params.polynomial_order = static_cast<int>(value[POLYNOMIAL_ORDER]);
  }
  catch (const XmlRpc::XmlRpcException& ex)
  {
    CONSOLE_BRIDGE_logError("Error configuring MLS Smoothing Filter: '%s'", ex.getMessage().c_str());
    return false;
  }

  return true;
}

template <typename PointT>
bool MLSSmoothingFilter<PointT>::filter(const T& input, T& output)
{
  // Convert the input cloud to XYZ type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(input, *cloud);

  typename pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setInputCloud(cloud);
  mls.setSearchMethod(tree);

  mls.setSearchRadius(params.search_radius);
  mls.setSqrGaussParam(params.search_radius * params.search_radius);
  mls.setPolynomialOrder(params.polynomial_order);

  // Set the computation of normals false since we are only instantiating PCL XYZ point types as inputs/outputs
  mls.setComputeNormals(false);

  // Run the filter
  pcl::PointCloud<pcl::PointNormal> filtered_cloud;
  mls.process(filtered_cloud);

  // Convert the output cloud back into the template format
  pcl::copyPointCloud(filtered_cloud, output);

  return true;
}

/**
 * Provide an override specifically for the Point Normal template type that will calculate normals
 */
template <>
bool MLSSmoothingFilter<pcl::PointNormal>::filter(const pcl::PointCloud<pcl::PointNormal>& input,
                                                  pcl::PointCloud<pcl::PointNormal>& output)
{
  // Convert the input cloud to XYZ type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(input, *cloud);

  typename pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setInputCloud(cloud);
  mls.setSearchMethod(tree);

  mls.setSearchRadius(params.search_radius);
  mls.setSqrGaussParam(params.search_radius * params.search_radius);
  mls.setPolynomialOrder(params.polynomial_order);

  // Set the computation of normals true since the incoming cloud has normals
  mls.setComputeNormals(true);

  // Run the filter
  mls.process(output);

  return true;
}

template <typename PointT>
std::string MLSSmoothingFilter<PointT>::getName() const
{
  return utils::getClassName<decltype(*this)>();
}

}  // namespace cloud
}  // namespace noether_filtering

#define PCL_INSTANTIATE_MLSSmoothingFilter(T)                                                                          \
  template class PCL_EXPORTS noether_filtering::cloud::MLSSmoothingFilter<T>;

#endif  // NOETHER_FILTERING_CLOUD_IMPL_MLS_SMOOTHING_FILTER_HPP
