/*
 * filter_manager.hpp
 *
 *  Created on: Oct 14, 2019
 *      Author: jrgnicho
 */
#ifndef INCLUDE_NOETHER_FILTERING_FILTER_GROUP_H_
#define INCLUDE_NOETHER_FILTERING_FILTER_GROUP_H_

#include <string>
#include <memory>
#include <XmlRpcValue.h>
#include <pluginlib/class_loader.h>
#include "noether_filtering/filter_base.h"

namespace noether_filtering
{
namespace config_fields
{
namespace filter
{
static const std::string TYPE_NAME = "type";
static const std::string NAME = "name";
static const std::string CONFIG = "config";
}  // namespace filter

namespace group
{
static const std::string GROUP_NAME = "group_name";
static const std::string CONTINUE_ON_FAILURE = "continue_on_failure";
static const std::string VERBOSITY_ON = "verbosity_on";
static const std::string FILTERS = "filters";
}  // namespace group
}  // namespace config_fields

/**
 * @brief The FilterManager class
 */
template <class F>
class FilterGroup
{
public:
  FilterGroup(const std::string& base_class_name);
  virtual ~FilterGroup() = default;

  /**
   * @details Initializes the filter chain and loads all the filter plugins from a yaml structured parameter.
   * When 'continue_on_failure = true' then it only takes one filter to succeed for the whole pass through the group to
   * succeed. When 'continue_on_failure = false' then the algorithm returns false as soon as one filter fails. The
   * parameter must conform to the following syntax:
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
  bool applyFilters(const F& input, F& output, std::string& err_msg);
  bool applyFilters(const std::vector<std::string>& filters, const F& input, F& output, std::string& err_msg);

protected:
  using FilterT = FilterBase<F>;
  typedef typename std::unique_ptr<FilterT> FilterBasePtr;

  pluginlib::ClassLoader<FilterT> filter_loader_;
  std::vector<std::string> filters_loaded_;
  std::map<std::string, FilterBasePtr> filters_map_;
  bool continue_on_failure_;
  bool verbosity_on_;
};

} /* namespace noether_filtering */

#endif /* INCLUDE_NOETHER_FILTERING_FILTER_GROUP_H_ */
