/**
 * @author Jorge Nicho
 * @file mesh_boundary_finder.cpp
 * @date Dec 5, 2019
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <boost/bimap.hpp>
#include <boost/make_shared.hpp>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <console_bridge/console.h>
#include <noether_conversions/noether_conversions.h>
#include <tool_path_planner/half_edge_boundary_finder.h>
#include <tool_path_planner/utilities.h>


using MeshTraits = pcl::geometry::DefaultMeshTraits<std::size_t>;
typedef pcl::geometry::TriangleMesh< MeshTraits > TraingleMesh;

boost::optional<TraingleMesh> createTriangleMesh(const pcl::PolygonMesh& input_mesh)
{
  std::stringstream ss;
  TraingleMesh mesh;
  typedef boost::bimap<uint32_t, TraingleMesh::VertexIndex> MeshIndexMap;
  MeshIndexMap mesh_index_map;
  for (size_t ii = 0; ii < input_mesh.polygons.size(); ++ii)
  {
    const std::vector<uint32_t>& vertices = input_mesh.polygons.at(ii).vertices;
    if (vertices.size() != 3)
    {
      ss.str("");
      ss << "Found polygon with " << vertices.size() << " sides, only triangle mesh supported!";
      CONSOLE_BRIDGE_logInform(ss.str().c_str());
      return boost::none;
    }
    TraingleMesh::VertexIndices vi;
    for (std::vector<uint32_t>::const_iterator vidx = vertices.begin(), viend = vertices.end();
         vidx != viend; ++vidx)
    {
      //      mesh_index_map2.left.count
      if (!mesh_index_map.left.count(*vidx))
      {
        mesh_index_map.insert(MeshIndexMap::value_type(*vidx, mesh.addVertex(*vidx)));
      }
      vi.push_back(mesh_index_map.left.at(*vidx));
    }
    mesh.addFace(vi.at(0), vi.at(1), vi.at(2));
  }

  return mesh;
}

/** \brief Get a collection of boundary half-edges for the input mesh.
  * \param[in] mesh The input mesh.
  * \param[out] boundary_he_collection Collection of boundary half-edges. Each element in the vector
 * is one connected boundary. The whole boundary is the union of all elements.
  * \param [in] expected_size If you already know the size of the longest boundary you can tell this
 * here. Defaults to 3 (minimum possible boundary).
  * \author Martin Saelzle
  * \ingroup geometry
  */
template <class MeshT>
void getBoundBoundaryHalfEdges(const MeshT& mesh,
                               std::vector<typename MeshT::HalfEdgeIndices>& boundary_he_collection,
                               const size_t expected_size = 3)
{
  typedef MeshT Mesh;
  typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
  typedef typename Mesh::HalfEdgeIndices HalfEdgeIndices;
  typedef typename Mesh::InnerHalfEdgeAroundFaceCirculator IHEAFC;

  boundary_he_collection.clear();

  HalfEdgeIndices boundary_he;
  boundary_he.reserve(expected_size);
  std::vector<bool> visited(mesh.sizeEdges(), false);
  IHEAFC circ, circ_end;

  for (HalfEdgeIndex i(0); i < HalfEdgeIndex(mesh.sizeHalfEdges()); ++i)
  {
    if (mesh.isBoundary(i) && !visited[pcl::geometry::toEdgeIndex(i).get()])
    {
      boundary_he.clear();

      circ = mesh.getInnerHalfEdgeAroundFaceCirculator(i);
      circ_end = circ;
      do
      {
        visited[pcl::geometry::toEdgeIndex(circ.getTargetIndex()).get()] = true;
        boundary_he.push_back(circ.getTargetIndex());
      } while (++circ != circ_end);

      boundary_he_collection.push_back(boundary_he);
    }
  }
}

bool decimate(const pcl::PointCloud<pcl::PointNormal>& in, pcl::PointCloud<pcl::PointNormal>& out, double min_point_dist)
{
   out.clear();
   out.push_back(in[0]);
   using PType = std::remove_reference<decltype(in)>::type::PointType;
   for(std::size_t i = 1 ; i < in.size() - 1; i++)
   {
     const PType& p = in[i];

     double dist = (out.back().getVector3fMap() - p.getVector3fMap()).norm();
     if(dist > min_point_dist)
     {
       out.push_back(p);
     }
   }
   out.push_back(in.back());
   return out.size() > 2;
}

namespace tool_path_planner
{
HalfEdgeBoundaryFinder::HalfEdgeBoundaryFinder()
{

}

HalfEdgeBoundaryFinder::~HalfEdgeBoundaryFinder()
{

}

void HalfEdgeBoundaryFinder::setInput(pcl::PolygonMesh::ConstPtr mesh)
{
  mesh_ = mesh;
}

void HalfEdgeBoundaryFinder::setInput(const shape_msgs::Mesh& mesh)
{
  pcl::PolygonMesh::Ptr pcl_mesh = boost::make_shared<pcl::PolygonMesh>();
  noether_conversions::convertToPCLMesh(mesh,*pcl_mesh);
  setInput(pcl_mesh);
}

boost::optional<std::vector<geometry_msgs::PoseArray> >
HalfEdgeBoundaryFinder::generate(const tool_path_planner::HalfEdgeBoundaryFinder::Config& config)
{
  using namespace pcl;
  CONSOLE_BRIDGE_logInform("Input mesh has %lu polygons",mesh_->polygons.size());
  boost::optional<TraingleMesh> mesh = createTriangleMesh(*mesh_);
  if(!mesh)
  {
    CONSOLE_BRIDGE_logError("Failed to create triangle mesh");
    return boost::none;
  }

  if(mesh->getVertexDataCloud().empty())
  {
    CONSOLE_BRIDGE_logError("Generated Triangle mesh contains no data");
    return boost::none;
  }

  std::vector<TraingleMesh::HalfEdgeIndices> boundary_he_indices;
  getBoundBoundaryHalfEdges(mesh.get(), boundary_he_indices);

  // initializing octree
  PointCloud<PointNormal>::Ptr input_cloud = boost::make_shared<PointCloud<PointNormal>>();
  PointCloud<PointXYZ>::Ptr input_points = boost::make_shared<PointCloud<PointXYZ>>();
  noether_conversions::convertToPointNormals(*mesh_, *input_cloud);
  pcl::copyPointCloud(*input_cloud, *input_points );


  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (input_points);

  // traversing half edges list
  std::vector<geometry_msgs::PoseArray> boundary_poses;


  for (std::vector<TraingleMesh::HalfEdgeIndices>::const_iterator boundary = boundary_he_indices.begin(),
                                                                  b_end = boundary_he_indices.end();
       boundary != b_end; ++boundary)
  {
    geometry_msgs::PoseArray bound_segment_poses;
    PointCloud<PointNormal> bound_segment_points;
    for (TraingleMesh::HalfEdgeIndices::const_iterator edge = boundary->begin(), edge_end = boundary->end();
         edge != edge_end; ++edge)
    {
      TraingleMesh::VertexIndex vertex = mesh->getOriginatingVertexIndex(*edge);
      if(mesh->getVertexDataCloud().size() <= vertex.get())
      {
        CONSOLE_BRIDGE_logError("Vertex index exceeds size of vertex list");
        return boost::none;
      }
      std::size_t source_idx = mesh->getVertexDataCloud()[vertex.get()];

      if(input_cloud->size() <= source_idx)
      {
        CONSOLE_BRIDGE_logError("Point index exceeds size of mesh point cloud");
        return boost::none;
      }

      bound_segment_points.push_back((*input_cloud)[source_idx]);
    }

    if(config.min_point_dist >  1e-6 )
    {

      decltype(bound_segment_points) decimated_points;
      if(!decimate(bound_segment_points, decimated_points, config.min_point_dist))
      {
        CONSOLE_BRIDGE_logDebug("Decimation failed, ignoring segment");
        continue;
      }
      bound_segment_points = std::move(decimated_points);
    }

    if(!createPoseArray(bound_segment_points,{},bound_segment_poses))
    {
      return boost::none;
    }

    boundary_poses.push_back(bound_segment_poses);
  }

  // sorting
  CONSOLE_BRIDGE_logInform("Found %lu boundary segments, removing invalid ones", boundary_poses.size());
  std::sort(boundary_poses.begin(), boundary_poses.end(),[](decltype(boundary_poses)::value_type& p1,
      decltype(boundary_poses)::value_type& p2){
    return p1.poses.size() >  p2.poses.size();
  });

  // erase
  decltype(boundary_poses)::iterator last = std::remove_if(boundary_poses.begin(), boundary_poses.end(),
                                                           [&config](decltype(boundary_poses)::value_type& p){
    return p.poses.size() < config.min_num_points;
  });
  boundary_poses.erase(last,boundary_poses.end());
  if(boundary_poses.empty())
  {
    CONSOLE_BRIDGE_logError("No valid boundary segments were found, original mesh may have duplicate vertices");
    return boost::none;
  }

  CONSOLE_BRIDGE_logInform("Found %lu valid boundary segments", boundary_poses.size());
  return boundary_poses;
}

} /* namespace tool_path_planner */
