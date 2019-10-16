/*
 * mesh_filter_base.h
 *
 *  Created on: Oct 9, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_NOETHER_FILTERING_FILTER_BASE_H_
#define INCLUDE_NOETHER_FILTERING_FILTER_BASE_H_

#include <XmlRpcValue.h>

namespace noether_filtering
{

template<class F>
class FilterBase
{
public:
  FilterBase()
  {

  }

  virtual ~FilterBase()
  {

  }

  virtual bool configure(XmlRpc::XmlRpcValue config) = 0;
  virtual bool filter(const F& in, F& out) = 0;
  virtual std::string getName() = 0;

};
}



#endif /* INCLUDE_NOETHER_FILTERING_FILTER_BASE_H_ */
