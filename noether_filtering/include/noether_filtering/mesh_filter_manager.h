/*
 * filter_manager.h
 *
 *  Created on: Oct 10, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_NOETHER_FILTERING_MESH_FILTER_MANAGER_H_
#define INCLUDE_NOETHER_FILTERING_MESH_FILTER_MANAGER_H_

#include <pluginlib/class_loader.h>
#include "noether_filtering/mesh_filter_base.h"

namespace noether_filtering
{
struct MeshFilterInfo
{
  std::string type_name;      /** @brief the filter type */
  std::string name;           /** @brief an alias for the filter */
  XmlRpc::XmlRpcValue config; /** @brief the filter configuration */
};

class MeshFilterManager
{
public:
  MeshFilterManager();
  virtual ~MeshFilterManager();

  /**
   * @details Initializes the manager and loads all the filter plugins from a yaml structured parameter
   * The parameter must conform to the following syntax:
   *
   * continue_on_failure: True
   * mesh_filters:
   * - type: DemoFilter1
   *   name: demo_filter_1
   *   config:
   *    param1: 20
   *    param2: 'optimize'
   *      .
   *      .
   *      .
   * - type: DemoFilter2
   *   name: demo_filter_2
   *   config:
   *    x: 1.0
   *    y: 3.5
   *      .
   *      .
   *      .
   * @param config
   * @return
   */
  bool init(XmlRpc::XmlRpcValue config);
  bool applyFilters(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out, std::string& err_msg);
  bool applyFilters(const std::vector<std::string>& filters, const pcl::PolygonMesh& mesh_in,
                    pcl::PolygonMesh& mesh_out, std::string& err_msg);

protected:

  std::shared_ptr< pluginlib::ClassLoader<MeshFilterBase> > filter_loader_;
  std::vector<std::string> filters_loaded_;
  std::map<std::string, std::unique_ptr<MeshFilterBase>> filters_map_;
  bool continue_on_failure_;
  
};

} /* namespace noether_filtering */

#endif /* INCLUDE_NOETHER_FILTERING_MESH_FILTER_MANAGER_H_ */
