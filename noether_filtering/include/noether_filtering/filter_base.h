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
template <typename T>
class FilterBase
{
public:
  FilterBase() = default;
  virtual ~FilterBase() = default;

  virtual bool configure(XmlRpc::XmlRpcValue config) = 0;
  virtual bool filter(const T& input, T& output) = 0;
  virtual std::string getName() const = 0;
};

}  // namespace noether_filtering

#endif /* INCLUDE_NOETHER_FILTERING_MESH_FILTER_BASE_H_ */
