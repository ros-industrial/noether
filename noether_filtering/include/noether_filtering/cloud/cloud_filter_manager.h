#ifndef NOETHER_FILTERING_CLOUD_CLOUD_FILTER_MANAGER_H
#define NOETHER_FILTERING_CLOUD_CLOUD_FILTER_MANAGER_H

#include "noether_filtering/filter_manager.h"
#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>

// Forward declare PCL PointCloud
namespace pcl
{
template<typename PointT>
class PointCloud;
}

namespace noether_filtering
{

template<typename PointT>
using CloudFilterGroup = FilterManager<typename boost::shared_ptr<pcl::PointCloud<PointT>>>;

template<typename PointT>
using CloudFilterManager = FilterManager<typename boost::shared_ptr<pcl::PointCloud<PointT>>>;

} // namespace noether_filtering

#endif // NOETHER_FILTERING_CLOUD_CLOUD_FILTER_MANAGER_H
