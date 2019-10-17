/*
 * filter_manager.h
 *
 *  Created on: Oct 10, 2019
 *      Author: jrgnicho
 */
#ifndef NOETHER_FILTERING_FILTER_MANAGER_H
#define NOETHER_FILTERING_FILTER_MANAGER_H

#include "noether_filtering/filter_group.h"

namespace noether_filtering
{
namespace config_fields
{
namespace manager
{
static const std::string FILTER_GROUPS = "filter_groups";
} // namespace manager
} // namespace config_fields

/**
 * @brief Filter Manager class
 */
template<typename T>
class FilterManager
{
public:
  FilterManager() = default;
  virtual ~FilterManager() = default;

  /**
   * @details loads several filter chain configurations. The yaml structure is expected to be as follows:
   * filter_groups:
   * - group_name: DEFAULT
   *   continue_on_failure: False
   *   mesh_filters:
   *   - type: DemoFilter1
   *     name: demo_filter_1
   *     config:
   *      param1: 20
   *      param2: 'optimize'
   *      .
   *      .
   *      .
   *   - type: DemoFilter2
   *     name: demo_filter_2
   *     config:
   *      x: 1.0
   *      y: 3.5
   * - group_name: GROUP_1
   *   continue_on_failure: False
   *   mesh_filters:
   *   - type: DemoFilterX
   *     name: demo_filter_x
   *     config:
   *      param_x: 20
   *      .
   *      .
   *      .
   *   - type: DemoFilterY
   *     name: demo_filter_y
   *     config:
   *      a: True
   *      b: 20
   *   .
   *   .
   *   .
   *
   * @param config The configuration
   * @return True on success, false otherwise
   */
  bool init(XmlRpc::XmlRpcValue config);

  /**
   * @brief gets the requested filter group
   * @param name the name of the group
   * @return the requested filter group, returns a nullptr when the requested group isn't recognized
   */
  std::shared_ptr<FilterGroup<T>> getFilterGroup(const std::string& name) const;

protected:
  std::map<std::string, std::shared_ptr<FilterGroup<T>>> filter_groups_map_;
};

} /* namespace noether_filtering */

#endif /* NOETHER_FILTERING_FILTER_MANAGER_H */
