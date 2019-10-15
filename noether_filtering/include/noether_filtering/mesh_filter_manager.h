/*
 * filter_manager.h
 *
 *  Created on: Oct 10, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_NOETHER_FILTERING_MESH_FILTER_MANAGER_H_
#define INCLUDE_NOETHER_FILTERING_MESH_FILTER_MANAGER_H_

#include <pluginlib/class_loader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "noether_filtering/filter_group.hpp"
#include "noether_filtering/mesh_filter_base.h"


namespace noether_filtering
{

using MeshFilterGroup = FilterGroup<pcl::PolygonMesh>;
class MeshFilterManager
{
public:
  MeshFilterManager();
  virtual ~MeshFilterManager();

 /**
   * @details Multiple mesh filter groups each with their own configuration
   *
   * @param config
   * @return
   */
  bool init(XmlRpc::XmlRpcValue config);

  /**
   * @brief gets the requested filter group
   * @param name
   * @return
   */
  std::shared_ptr<MeshFilterGroup> getFilterGroup(const std::string& name);

protected:
  
  std::map<std::string, std::shared_ptr<MeshFilterGroup> > filter_groups_map_;


};

} /* namespace noether_filtering */

#endif /* INCLUDE_NOETHER_FILTERING_MESH_FILTER_MANAGER_H_ */
