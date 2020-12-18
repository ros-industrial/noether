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
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkParametricSpline.h>
#include <vtkShadowMapPass.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <console_bridge/console.h>
#include <noether_conversions/noether_conversions.h>
#include <tool_path_planner/halfedge_edge_generator.h>
#include <tool_path_planner/utilities.h>
#include <numeric>

static const double EPSILON = 1e-3;
static const double MIN_POINT_DIST_ALLOWED = 1e-8;

using MeshTraits = pcl::geometry::DefaultMeshTraits<std::size_t>;
typedef pcl::geometry::TriangleMesh<MeshTraits> TraingleMesh;

boost::optional<TraingleMesh> createTriangleMesh(const pcl::PolygonMesh& input_mesh)
{
  std::stringstream ss;
  TraingleMesh mesh;
  typedef boost::bimap<uint32_t, TraingleMesh::VertexIndex> MeshIndexMap;
  MeshIndexMap mesh_index_map;
  for (const pcl::Vertices& plg : input_mesh.polygons)
  {
    const std::vector<uint32_t>& vertices = plg.vertices;
    if (vertices.size() != 3)
    {
      ss.str("");
      ss << "Found polygon with " << vertices.size() << " sides, only triangle mesh supported!";
      CONSOLE_BRIDGE_logInform(ss.str().c_str());
      return boost::none;
    }
    TraingleMesh::VertexIndices vi;
    for (std::vector<uint32_t>::const_iterator vidx = vertices.begin(), viend = vertices.end(); vidx != viend; ++vidx)
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

/**
 * @brief Enforces a minimum distance between adjacent points
 * @param in              The input cloud
 * @param out             The output cloud
 * @param min_point_dist  The minimum distance between adjacent points
 * @return  True on success, False when resulting cloud has less than 2 points.
 */
bool applyMinPointDistance(const pcl::PointCloud<pcl::PointNormal>& in,
                           pcl::PointCloud<pcl::PointNormal>& out,
                           double min_point_dist)
{
  out.clear();
  out.push_back(in[0]);
  using PType = std::remove_reference<decltype(in)>::type::PointType;
  for (std::size_t i = 1; i < in.size() - 1; i++)
  {
    const PType& p = in[i];

    double dist = (out.back().getVector3fMap() - p.getVector3fMap()).norm();
    if (dist > min_point_dist)
    {
      out.push_back(p);
    }
  }
  out.push_back(in.back());
  return out.size() > 2;
}

/**
 * @brief forces the required point spacing by computing new points along the curve
 * @param in    The input cloud
 * @param out   The output cloud
 * @param dist  The required point spacing distance
 * @return True on success, False otherwise
 */
bool applyEqualDistance(const pcl::PointCloud<pcl::PointNormal>& in,
                        pcl::PointCloud<pcl::PointNormal>& out,
                        double dist)
{
  using namespace pcl;
  using namespace Eigen;

  // kd tree will be used to recover the normals
  PointCloud<PointXYZ>::Ptr in_points = boost::make_shared<PointCloud<PointXYZ>>();
  copyPointCloud(in, *in_points);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(in_points);
  kdtree.setEpsilon(EPSILON);

  std::remove_reference<decltype(in)>::type::PointType new_pn;
  PointXYZ p_start, p_mid, p_end;
  Vector3f v_1, v_2, unitv_2, v_next;
  p_start = (*in_points)[0];
  p_mid = (*in_points)[1];
  v_1 = p_mid.getVector3fMap() - p_start.getVector3fMap();
  out.push_back(in[0]);  // adding first points
  for (std::size_t i = 2; i < in_points->size(); i++)
  {
    p_end = (*in_points)[i];
    while (dist < v_1.norm())
    {
      p_start.getVector3fMap() = p_start.getVector3fMap() + dist * v_1.normalized();
      std::tie(new_pn.x, new_pn.y, new_pn.z) = std::make_tuple(p_start.x, p_start.y, p_start.z);
      out.push_back(new_pn);
      v_1 = p_mid.getVector3fMap() - p_start.getVector3fMap();
    }

    v_2 = p_end.getVector3fMap() - p_mid.getVector3fMap();
    if (dist < (v_1 + v_2).norm())
    {
      // solve for x such that " x^2 + 2 * x * dot(v_1, unitv_2) + norm(v_1)^2 - d^2 = 0"
      unitv_2 = v_2.normalized();
      double b = 2 * v_1.dot(unitv_2);
      double c = std::pow(v_1.norm(), 2) - std::pow(dist, 2);
      double sqrt_term = std::sqrt(std::pow(b, 2) - 4 * c);
      double x = 0.5 * (-b + sqrt_term);
      x = (x > 0.0 && x < dist) ? x : 0.5 * (-b - sqrt_term);
      if (x > dist)
      {
        return false;
      }
      v_next = v_1 + x * unitv_2;

      // computing next point at distance "dist" from previous that lies on the curve
      p_start.getVector3fMap() = p_start.getVector3fMap() + v_next;
      std::tie(new_pn.x, new_pn.y, new_pn.z) = std::make_tuple(p_start.x, p_start.y, p_start.z);
      out.push_back(new_pn);
    }

    // computing values for next iter
    p_mid = p_end;
    v_1 = p_mid.getVector3fMap() - p_start.getVector3fMap();
  }

  // add last point if not already there
  if ((out.back().getVector3fMap() - in.back().getNormalVector3fMap()).norm() > MIN_POINT_DIST_ALLOWED)
  {
    out.push_back(in.back());
  }

  if (out.size() < 2)
  {
    CONSOLE_BRIDGE_logError("Points in curve segment are too close together");
    return false;
  }

  // recovering normals now
  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;
  PointXYZ query_point;
  for (auto& p : out)
  {
    k_indices.resize(1);
    k_sqr_distances.resize(1);
    std::tie(query_point.x, query_point.y, query_point.z) = std::make_tuple(p.x, p.y, p.z);
    if (kdtree.nearestKSearch(query_point, 1, k_indices, k_sqr_distances) < 1)
    {
      CONSOLE_BRIDGE_logError("No nearest points found");
      return false;
    }

    const auto& pnormal = in[k_indices.front()];
    std::tie(p.normal_x, p.normal_y, p.normal_z) =
        std::make_tuple(pnormal.normal_x, pnormal.normal_y, pnormal.normal_z);
  }
  return true;
}

/**
 * @brief uses a parametric spline to compute new points along the curve
 * @param in    The input cloud
 * @param out   The output cloud
 * @param dist  The desired distance between adjacent points
 * @return True on success, False otherwise
 */
bool applyParametricSpline(const pcl::PointCloud<pcl::PointNormal>& in,
                           pcl::PointCloud<pcl::PointNormal>& out,
                           double dist)
{
  using namespace pcl;

  // kd tree will be used to recover the normals
  PointCloud<PointXYZ>::Ptr in_points = boost::make_shared<PointCloud<PointXYZ>>();
  copyPointCloud(in, *in_points);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(in_points);
  kdtree.setEpsilon(EPSILON);

  // create vtk points array and computing total length
  vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
  auto prev_point = in.front();
  double total_length = 0;
  for (const auto& p : in)
  {
    double d = (p.getVector3fMap() - prev_point.getVector3fMap()).norm();
    total_length += d;
    vtk_points->InsertNextPoint(p.x, p.y, p.z);
    prev_point = p;
  }

  CONSOLE_BRIDGE_logDebug("Total boundary length = %f", total_length);

  // create spline
  vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
  spline->SetPoints(vtk_points);
  spline->ParameterizeByLengthOff();
  spline->ClosedOff();

  // interpolating
  double u[3], pt[3], du[9];

  double incr = dist / total_length;
  std::size_t num_points = std::ceil(total_length / dist) + 1;
  std::remove_reference<decltype(out)>::type::PointType new_point;
  PointXYZ query_point;
  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;
  out.reserve(num_points);
  out.push_back(in.front());
  prev_point = out.front();
  for (std::size_t i = 1; i < num_points; i++)
  {
    double interv = incr * i;
    interv = interv > 1.0 ? 1.0 : interv;
    std::tie(u[0], u[1], u[2]) = std::make_tuple(interv, interv, interv);
    spline->Evaluate(u, pt, du);
    std::tie(new_point.x, new_point.y, new_point.z) = std::make_tuple(pt[0], pt[1], pt[2]);
    std::tie(query_point.x, query_point.y, query_point.z) = std::make_tuple(pt[0], pt[1], pt[2]);

    double d = (new_point.getVector3fMap() - prev_point.getVector3fMap()).norm();
    if (d < MIN_POINT_DIST_ALLOWED)
    {
      // it's likely the same point, only expect this to occur at the last point
      continue;
    }

    // recovering normal now
    k_indices.resize(1);
    k_sqr_distances.resize(1);
    if (kdtree.nearestKSearch(query_point, 1, k_indices, k_sqr_distances) < 1)
    {
      CONSOLE_BRIDGE_logError("No nearest points found");
      return false;
    }

    const auto& pnormal = in[k_indices.front()];
    std::tie(new_point.normal_x, new_point.normal_y, new_point.normal_z) =
        std::make_tuple(pnormal.normal_x, pnormal.normal_y, pnormal.normal_z);
    std::cout << "New point is " << d << " from the previous one" << std::endl;
    out.push_back(new_point);
    prev_point = out.back();
  }
  CONSOLE_BRIDGE_logDebug("Parametric spline computed %lu  points", out.size());
  return true;
}

/**
 * @brief improves the normals by computing the average of the normals of the neighboring points
 * @param src     The source point cloud with point normals
 * @param src_xyz The same point cloud with just the points, passing it this way avoids making unnecessary copies
 * @param subset  The point cloud with normals that are to be improved
 * @param radius  The neighborhood search radius used by kdtree
 * @param weight  More distant points will have less influence on the final average value, set to 0 in order to disable
 * weighting
 */
void averageNormals(pcl::PointCloud<pcl::PointNormal>::ConstPtr src,
                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr src_xyz,
                    pcl::PointCloud<pcl::PointNormal>& subset,
                    double radius,
                    double weight)
{
  using namespace pcl;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(src_xyz);
  kdtree.setEpsilon(EPSILON);
  weight = std::abs(weight) > 1.0 ? 1.0 : std::abs(weight);
  double r_2 = pow(radius, 2);
  for (auto& p : subset)
  {
    std::vector<int> nearest_idx;
    std::vector<float> nearest_dist_sqr;
    PointXYZ pxyz;
    pxyz.getVector3fMap() = p.getVector3fMap();
    kdtree.radiusSearch(pxyz, radius, nearest_idx, nearest_dist_sqr);

    // adding normals
    std::size_t i = 0;
    Eigen::Vector3f average_normal = std::accumulate(
        nearest_idx.begin(),
        nearest_idx.end(),
        p.getNormalVector3fMap(),
        [&i, &src, &nearest_dist_sqr, &weight, &r = radius, &r_2](Eigen::Vector3f n, int idx) -> Eigen::Vector3f {
          if (idx > src->size())
          {
            return n;
          }

          double d_2 = nearest_dist_sqr[i++];
          double f;
          if (d_2 < 0.0 || d_2 > r_2)
          {
            f = 0.0;
          }
          else
          {
            f = (r - weight * std::sqrt(d_2)) / r;
          }

          Eigen::Vector3f v = (*src)[idx].getNormalVector3fMap();
          v *= (f > 0 ? f : 0);

          return n + v;
        });
    p.getNormalVector3fMap() = average_normal.normalized();
  }
}

namespace tool_path_planner
{
void HalfedgeEdgeGenerator::setConfiguration(const Config& config) { config_ = config; }

void HalfedgeEdgeGenerator::setInput(pcl::PolygonMesh::ConstPtr mesh) { mesh_ = mesh; }

void HalfedgeEdgeGenerator::setInput(vtkSmartPointer<vtkPolyData> mesh)
{
  if (!vtk_mesh_)
    vtk_mesh_ = vtkSmartPointer<vtkPolyData>::New();

  vtk_mesh_->DeepCopy(mesh);

  auto pcl_mesh = boost::make_shared<pcl::PolygonMesh>();
  pcl::VTKUtils::vtk2mesh(vtk_mesh_, *pcl_mesh);

  mesh_ = pcl_mesh;
}

void HalfedgeEdgeGenerator::setInput(const shape_msgs::Mesh& mesh)
{
  pcl::PolygonMesh::Ptr pcl_mesh = boost::make_shared<pcl::PolygonMesh>();
  noether_conversions::convertToPCLMesh(mesh, *pcl_mesh);
  setInput(pcl_mesh);
}

vtkSmartPointer<vtkPolyData> HalfedgeEdgeGenerator::getInput() { return vtk_mesh_; }

std::string HalfedgeEdgeGenerator::getName() const { return getClassName<decltype(*this)>(); }

boost::optional<ToolPaths> HalfedgeEdgeGenerator::generate()
{
  using namespace pcl;
  CONSOLE_BRIDGE_logInform("Input mesh has %lu polygons", mesh_->polygons.size());
  boost::optional<TraingleMesh> mesh = createTriangleMesh(*mesh_);
  if (!mesh)
  {
    CONSOLE_BRIDGE_logError("Failed to create triangle mesh");
    return boost::none;
  }

  if (mesh->getVertexDataCloud().empty())
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
  pcl::copyPointCloud(*input_cloud, *input_points);

  // traversing half edges list
  ToolPaths edge_paths;
  for (std::vector<TraingleMesh::HalfEdgeIndices>::const_iterator boundary = boundary_he_indices.begin(),
                                                                  b_end = boundary_he_indices.end();
       boundary != b_end;
       ++boundary)
  {
    PointCloud<PointNormal> bound_segment_points;
    for (TraingleMesh::HalfEdgeIndices::const_iterator edge = boundary->begin(), edge_end = boundary->end();
         edge != edge_end;
         ++edge)
    {
      TraingleMesh::VertexIndex vertex = mesh->getOriginatingVertexIndex(*edge);
      if (mesh->getVertexDataCloud().size() <= vertex.get())
      {
        CONSOLE_BRIDGE_logError("Vertex index exceeds size of vertex list");
        return boost::none;
      }
      std::size_t source_idx = mesh->getVertexDataCloud()[vertex.get()];

      if (input_cloud->size() <= source_idx)
      {
        CONSOLE_BRIDGE_logError("Point index exceeds size of mesh point cloud");
        return boost::none;
      }

      bound_segment_points.push_back((*input_cloud)[source_idx]);
    }

    if (bound_segment_points.size() < config_.min_num_points)
    {
      continue;
    }

    // decimating
    CONSOLE_BRIDGE_logDebug("Found boundary with %lu points", bound_segment_points.size());
    if (config_.point_dist > MIN_POINT_DIST_ALLOWED)
    {
      decltype(bound_segment_points) decimated_points;

      switch (config_.point_spacing_method)
      {
        case PointSpacingMethod::NONE:
          decimated_points = bound_segment_points;
          break;

        case PointSpacingMethod::EQUAL_SPACING:
          if (!applyEqualDistance(bound_segment_points, decimated_points, config_.point_dist))
          {
            CONSOLE_BRIDGE_logError("applyEqualDistance point spacing method failed");
            return boost::none;
          }
          break;

        case PointSpacingMethod::MIN_DISTANCE:
          if (!applyMinPointDistance(bound_segment_points, decimated_points, config_.point_dist))
          {
            CONSOLE_BRIDGE_logError("applyMinPointDistance point spacing method failed");
            return boost::none;
          }
          break;

        case PointSpacingMethod::PARAMETRIC_SPLINE:
          if (!applyParametricSpline(bound_segment_points, decimated_points, config_.point_dist))
          {
            CONSOLE_BRIDGE_logError("applyParametricSpline point spacing method failed");
            return boost::none;
          }
          break;

        default:
          CONSOLE_BRIDGE_logError("The point spacing method %i is not valid",
                                  static_cast<int>(config_.point_spacing_method));
          return boost::none;
      }

      CONSOLE_BRIDGE_logDebug(
          "Boundary with %lu points was decimated to %lu points", bound_segment_points.size(), decimated_points.size());
      bound_segment_points = std::move(decimated_points);
    }

    if (bound_segment_points.size() < config_.min_num_points)
    {
      CONSOLE_BRIDGE_logDebug("Decimated boundary contains %lu points and does not meet the minimum of %lu, ignoring",
                              bound_segment_points.size(),
                              config_.min_num_points);
      continue;
    }

    // average normals
    if (config_.normal_averaging)
    {
      averageNormals(input_cloud,
                     input_points,
                     bound_segment_points,
                     config_.normal_search_radius,
                     config_.normal_influence_weight);
    }

    ToolPathSegment edge_segment;
    if (!createToolPathSegment(bound_segment_points, {}, edge_segment))
      return boost::none;

    edge_paths.push_back({ edge_segment });
    CONSOLE_BRIDGE_logInform("Added boundary with %lu points", edge_paths.back()[0].size());
  }

  // sorting
  std::sort(edge_paths.begin(),
            edge_paths.end(),
            [](decltype(edge_paths)::value_type& p1, decltype(edge_paths)::value_type& p2) {
              return p1[0].size() > p2[0].size();
            });

  // erase
  decltype(edge_paths)::iterator last =
      std::remove_if(edge_paths.begin(), edge_paths.end(), [&](decltype(edge_paths)::value_type& p) {
        return p[0].size() < config_.min_num_points;
      });
  edge_paths.erase(last, edge_paths.end());

  if (edge_paths.empty())
  {
    CONSOLE_BRIDGE_logError("No valid edge segments were found, original mesh may have duplicate vertices");
    return boost::none;
  }

  if (config_.max_segment_length > 0)
  {
    edge_paths = tool_path_planner::splitSegments(edge_paths, config_.max_segment_length);
  }

  CONSOLE_BRIDGE_logInform("Found %lu valid edge segments", edge_paths.size());

  return edge_paths;
}

} /* namespace tool_path_planner */
