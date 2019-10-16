/*
 * mesh_filter_base.h
 *
 *  Created on: Oct 9, 2019
 *      Author: jrgnicho
 */
#ifndef INCLUDE_NOETHER_FILTERING_MESH_FILTER_BASE_H_
#define INCLUDE_NOETHER_FILTERING_MESH_FILTER_BASE_H_

#include <memory>
#include <XmlRpcValue.h>

namespace noether_filtering
{

namespace config_field_names
{
static const std::string CONTINUE_ON_FAILURE = "continue_on_failure";
static const std::string FILTERS = "filters";
static const std::string TYPE_NAME = "type";
static const std::string NAME = "name";
static const std::string CONFIG = "config";
}

struct FilterInfo
{
  std::string type_name;      /** @brief the filter type */
  std::string name;           /** @brief an alias for the filter */
  XmlRpc::XmlRpcValue config; /** @brief the filter configuration */
};

template <typename T>
class FilterBase
{
public:
  FilterBase() = default;
  virtual ~FilterBase() = default;

  virtual bool configure(XmlRpc::XmlRpcValue config) = 0;
  virtual bool filter(const T& input, T& output) = 0;
  virtual std::string getName() = 0;
};

} // namespace noether_filtering

#endif /* INCLUDE_NOETHER_FILTERING_MESH_FILTER_BASE_H_ */
