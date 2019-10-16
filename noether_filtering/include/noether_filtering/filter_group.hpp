/*
 * filter_manager.hpp
 *
 *  Created on: Oct 14, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_NOETHER_FILTERING_FILTER_GROUP_HPP_
#define INCLUDE_NOETHER_FILTERING_FILTER_GROUP_HPP_

#include <string>
#include <memory>
#include <XmlRpcValue.h>
#include <pluginlib/class_loader.h>
#include "noether_filtering/filter_base.h"

namespace noether_filtering
{

template<class F>
class FilterGroup
{
public:

  using FilterT = FilterBase< F >;

  FilterGroup();
  virtual ~FilterGroup();
  
    /**
   * @details Initializes the filter chain and loads all the filter plugins from a yaml structured parameter
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
  bool applyFilters(const F& input, F& output, std::string& err_msg);
  bool applyFilters(const std::vector<std::string>& filters, const F& input, F& output, std::string& err_msg);

protected:
  std::shared_ptr< typename pluginlib::ClassLoader< FilterT > > filter_loader_;
  std::vector<std::string> filters_loaded_;
  std::map<std::string, std::unique_ptr< FilterT> > filters_map_;
  bool continue_on_failure_;
};


} /* namespace noether_filtering */

#include <../src/filter_group.cpp>
#endif /* INCLUDE_NOETHER_FILTERING_FILTER_GROUP_HPP_ */
