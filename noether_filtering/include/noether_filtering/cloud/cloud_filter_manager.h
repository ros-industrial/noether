#ifndef NOETHER_FILTERING_CLOUD_CLOUD_FILTER_MANAGER_H
#define NOETHER_FILTERING_CLOUD_CLOUD_FILTER_MANAGER_H

#include "noether_filtering/filter_manager.h"
#include <pcl/point_types.h>

// Forward declare PCL PointCloud
namespace pcl
{
template <typename PointT>
class PointCloud;
}

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
using CloudFilterGroup = FilterManager<pcl::PointCloud<PointT>>;

template <typename PointT>
using CloudFilterManager = FilterManager<pcl::PointCloud<PointT>>;

}  // namespace cloud
}  // namespace noether_filtering

#endif  // NOETHER_FILTERING_CLOUD_CLOUD_FILTER_MANAGER_H
