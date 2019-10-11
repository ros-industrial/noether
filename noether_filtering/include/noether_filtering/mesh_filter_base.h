/*
 * mesh_filter_base.h
 *
 *  Created on: Oct 9, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_NOETHER_FILTERING_MESH_FILTER_BASE_H_
#define INCLUDE_NOETHER_FILTERING_MESH_FILTER_BASE_H_

#include <XmlRpcValue.h>
#include <pcl/PolygonMesh.h>

namespace noether_filtering
{

class MeshFilterBase
{
public:
  MeshFilterBase()
  {

  }

  virtual ~MeshFilterBase()
  {

  }

  virtual bool configure(XmlRpc::XmlRpcValue config) = 0;
  virtual bool filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out) = 0;
  virtual std::string getName() = 0;

};
}



#endif /* INCLUDE_NOETHER_FILTERING_MESH_FILTER_BASE_H_ */
