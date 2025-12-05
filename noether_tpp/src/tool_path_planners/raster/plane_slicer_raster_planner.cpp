#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
#include <noether_tpp/utils.h>

#include <boost/graph/directed_graph.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <stdexcept>  // std::runtime_error
#include <utility>    // std::move()
#include <vector>     // std::vector
#include <set>
#include <stack>

#include <pcl/common/common.h>  // pcl::getMinMax3d()
#include <pcl/common/pca.h>     // pcl::PCA
#include <pcl/conversions.h>    // pcl::fromPCLPointCloud2
#include <pcl/io/vtk_lib_io.h>
#include <vtkAppendPolyData.h>
#include <vtkCutter.h>
#include <vtkErrorCode.h>
#include <vtkPlane.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkStripper.h>

namespace
{
std::vector<std::vector<vtkIdType>> fromVTK(vtkSmartPointer<vtkCellArray> lines)
{
  const std::size_t n_lines = lines->GetNumberOfCells();
  std::vector<std::vector<vtkIdType>> out(n_lines);

  lines->InitTraversal();
  for(std::size_t i = 0; i < n_lines; ++i)
  {
    vtkNew<vtkIdList> ids;
    lines->GetNextCell(ids);
    const std::size_t n_points = ids->GetNumberOfIds();

    out[i].resize(n_points);

    for(std::size_t j = 0; j < n_points; ++j)
    {
      out[i][j] = ids->GetId(j);
    }
  }

  return out;
}

/**
 * @details Finds the connecting vertex ID in two segments
 */
std::optional<vtkIdType> findConnectingVertex(const std::vector<vtkIdType>& s1,
                                              const std::vector<vtkIdType>& s2)
{
  // Convert to sets
  const std::set<vtkIdType> set_s1(s1.begin(), s1.end());
  const std::set<vtkIdType> set_s2(s2.begin(), s2.end());

  // Find the set intersection
  std::vector<vtkIdType> connections;
  std::set_intersection(set_s1.begin(), set_s1.end(), set_s2.begin(), set_s2.end(), std::back_inserter(connections));

  if (connections.size() > 1)
    throw std::runtime_error("Too many connecting vertices found between adjacent segments");

  if (connections.empty())
    return {};

  return connections[0];
}

using Vertex = std::pair<std::size_t, vtkIdType>;
using Graph = boost::directed_graph<Vertex>;
using VertexDescriptor = boost::graph_traits<Graph>::vertex_descriptor;

VertexDescriptor findVertexDescriptor(const Graph& g,
                                      const Vertex& vertex)
{
  Graph::vertex_iterator it, end;
  for(boost::tie(it, end) = boost::vertices(g); it != end; ++it)
  {
    if(g[*it] == vertex)
      return *it;
  }

  return Graph::null_vertex();
}

std::vector<std::vector<vtkIdType>> topologicalSort(const std::vector<std::vector<vtkIdType>>& segments)
{
  if (segments.size() <= 1)
    return segments;

  // Allocate the graph
  Graph graph;

  // Add the nodes (vertices in the segments) and the edge between them an the next nodes
  for(std::size_t i = 0; i < segments.size(); ++i)
  {
    VertexDescriptor v_prev = boost::add_vertex(std::make_pair(i, segments[i][0]), graph);

    for (std::size_t j = 1; j < segments[i].size(); ++j)
    {
      VertexDescriptor v_next = boost::add_vertex(std::make_pair(i, segments[i][j]), graph);
      boost::add_edge(v_prev, v_next, graph);

      v_prev = v_next;
    }
  }

  // Add edges between segments
  for(std::size_t i = 0; i < segments.size() - 1; ++i)
  {
    for (std::size_t j = i + 1; j < segments.size(); ++j)
    {
      std::optional<vtkIdType> intersection = findConnectingVertex(segments[i], segments[j]);
      if (intersection)
      {
        VertexDescriptor v1 = findVertexDescriptor(graph, std::make_pair(i, *intersection));
        VertexDescriptor v2 = findVertexDescriptor(graph, std::make_pair(j, *intersection));
        boost::add_edge(v1, v2, graph);
      }
    }
  }

  std::vector<VertexDescriptor> topo_sort;
  boost::topological_sort(graph, std::back_inserter(topo_sort));

  // Reduce all vertices to sorted segment indices
  std::vector<std::size_t> sorted_idx;
  for(const VertexDescriptor& v : topo_sort)
    if (std::find(sorted_idx.begin(), sorted_idx.end(), graph[v].first) == sorted_idx.end())
      sorted_idx.push_back(graph[v].first);

  // Use sorted segment indices to reconstruct output
  std::vector<std::vector<vtkIdType>> output;
  for(const std::size_t idx : sorted_idx)
    output.push_back(segments[idx]);

  return output;
}

void unifySegmentDirection(std::vector<std::vector<vtkIdType>>& segments,
                           vtkSmartPointer<vtkPoints> points,
                           const Eigen::Vector3d& reference_direction)
{
  for(auto s1 = segments.begin(); s1 < segments.end(); ++s1)
  {
    // Compute the direction vector between the first and second points in the segment
    const Eigen::Vector3d s1_pn1 = Eigen::Map<const Eigen::Vector3d>(points->GetPoint(*(s1->rbegin() + 1)));
    const Eigen::Vector3d s1_pn = Eigen::Map<const Eigen::Vector3d>(points->GetPoint(*(s1->rbegin())));
    const Eigen::Vector3d s1_dir = s1_pn - s1_pn1;

    // Reverse the order of segment 2 if it does not align with segment 1
    if (s1_dir.dot(reference_direction) < 0.0)
    {
      std::reverse(s1->begin(), s1->end());
    }
  }
}

std::vector<std::vector<vtkIdType>> joinContiguousSegments(const std::vector<std::vector<vtkIdType>> segments)
{
  if (segments.size() <= 1)
    return segments;

  // Initialize the output
  std::vector<std::vector<vtkIdType>> output;

  // Create a stack from the input segments (in reverse order, such that the first segment is at the top of the stack)
  std::stack<std::vector<vtkIdType>> stack(std::deque(segments.rbegin(), segments.rend()));

  while(!stack.empty())
  {
    // Get the first segment
    std::vector<vtkIdType> s1 = stack.top();
    stack.pop();

    // Check if stack is empty
    if (stack.empty())
    {
      output.push_back(s1);
      continue;
    }

    // Get the second segment
    std::vector<vtkIdType> s2 = stack.top();
    stack.pop();

    // Find the connecting vertex ID
    std::optional<vtkIdType> connecting_vertex = findConnectingVertex(s1, s2);

    // If no connection exists between the two segments, add the first segment to the output and return the second segment to the stack
    if (!connecting_vertex)
    {
      output.push_back(s1);
      stack.push(s2);
      continue;
    }

    // Find the iterator to the connection vertex in each segment
    auto it_s1 = std::find(s1.begin(), s1.end(), connecting_vertex);
    auto it_s2 = std::find(s2.begin(), s2.end(), connecting_vertex);

    /* We can only (easily) handle 4 cases for the location of the connecting vertex:
     *   1. End of segment 1, beginning of segment 2 -> add a new segment to the stack that is [s1, s2]
     *   2. Beginning of segment 1, end of segment 2 -> add a new segment to the stack that is [s2, s1]
     *   3. Beginning of segment 1, beginning of segment 2 -> add a new segment to the stack that is [reverse(s2), s1]
     *   4. End of segment 1, end of segment 2 -> add a new segment to the stack that is [s1, reverse(s2)]
     *
     * Note: when squashing segments into a new segment for the stack, we purposefully do not duplicate the shared vertex
     */
    if (it_s1 == s1.begin())
    {
      if (it_s2 == s2.end() - 1)
      {
        // Connection occurs at beginning of segment 1 and end of segment 2
        s2.insert(s2.end(), s1.begin() + 1, s1.end());
        stack.push(s2);
        continue;
      }
      else if(it_s2 == s2.begin())
      {
        s1.insert(s1.begin(), s2.rbegin(), s2.rend() - 1);
        stack.push(s1);
      }
      else
      {
        throw std::runtime_error("Connecting vertex exists at the beginning of one segment but not at the end or beginning of the adjacent segment");
      }
    }
    else if (it_s1 == s1.end() - 1)
    {
      if (it_s2 == s2.begin())
      {
        // Connection occurs at end of segment 1 and beginning of segment 2
        s1.insert(s1.end(), s2.begin() + 1, s2.end());
        stack.push(s1);
        continue;
      }
      else if(it_s2 == s2.end() - 1)
      {
        s1.insert(s1.end(), s2.rbegin() + 1, s2.rend());
        stack.push(s1);
      }
      else
      {
        throw std::runtime_error("Connecting vertex exists at the end of one segment but not at the beginning or end of the adjacent segment");
      }
    }
    else
    {
      throw std::runtime_error("Connecting vertex does not exist at the start or end of a segment");
    }
  }

  return output;
}

std::vector<std::vector<vtkIdType>> joinCloseSegments(const std::vector<std::vector<vtkIdType>>& segments,
                                                      const vtkSmartPointer<vtkPoints>& points,
                                                      const double max_separation_distance)
{
  std::vector<std::vector<vtkIdType>> output;

  std::stack stack(std::deque(segments.begin(), segments.end()));
  while(!stack.empty())
  {
    std::vector<vtkIdType> s1 = stack.top();
    stack.pop();

    if(stack.empty())
    {
      output.push_back(s1);
      continue;
    }

    std::vector<vtkIdType> s2 = stack.top();
    stack.pop();

    // Get the last point in segment 1
    const Eigen::Vector3d s1_end  = Eigen::Map<const Eigen::Vector3d>(points->GetPoint(*(s1.rbegin())));

    // Get the first point in segment 2
    const Eigen::Vector3d s2_begin = Eigen::Map<const Eigen::Vector3d>(points->GetPoint(*(s2.begin())));

    const double d = (s2_begin - s1_end).norm();
    if (d < max_separation_distance)
    {
      // Add segment 2 to segment 1 and push it onto the stack
      s1.insert(s1.end(), s2.begin(), s2.end());
      stack.push(s1);
    }
    else
    {
      // Add segment 1 to the output and return segment 2 to the stack
      output.push_back(s1);
      stack.push(s2);
    }
  }

  return output;
}

std::vector<std::vector<vtkIdType>> processPolyLines(vtkSmartPointer<vtkPolyData> polydata,
                                                     const Eigen::Vector3d& reference_direction,
                                                     const double min_hole_size)
{
  // Convert the lines into an easier format to work with
  std::vector<std::vector<vtkIdType>> segments = fromVTK(polydata->GetLines());

  // Topologically sort the segments such that connected segments (e.g., ones that share a common vertex) will be adjacent in the the container
  segments = topologicalSort(segments);

  // Ensure that all of the segments are pointing the same direction (relative to the first segment)
  unifySegmentDirection(segments, polydata->GetPoints(), reference_direction);

  // Join the contiguous segments
  segments = joinContiguousSegments(segments);

  // Join segments whose endpoints are within a specific distance of each other
  segments = joinCloseSegments(segments, polydata->GetPoints(), min_hole_size);

  return segments;
}

Eigen::Isometry3d createTransform(const Eigen::Vector3d& position,
                                  const Eigen::Vector3d& normal,
                                  const Eigen::Vector3d& path_dir)
{
  Eigen::Vector3d y = normal.cross(path_dir);;
  Eigen::Vector3d x = y.cross(normal);

  Eigen::Isometry3d transform;
  transform.matrix().col(0).head<3>() = x.normalized();
  transform.matrix().col(1).head<3>() = y.normalized();
  transform.matrix().col(2).head<3>() = normal.normalized();
  transform.matrix().col(3).head<3>() = position;

  return transform;
}

noether::ToolPath convertToPoses(vtkSmartPointer<vtkPolyData> polydata,
                                 const Eigen::Vector3d& reference_direction,
                                 const double min_hole_size)
{
  // Extract the point and normal data from the input polydata
  vtkSmartPointer<vtkPoints> points = polydata->GetPoints();
  vtkSmartPointer<vtkPointData> point_data = polydata->GetPointData();

  // Process the tool path lines to ensure proper segment ordering and connectivity
  std::vector<std::vector<vtkIdType>> segments = processPolyLines(polydata, reference_direction, min_hole_size);

  // Create the output container
  noether::ToolPath tool_path;
  tool_path.reserve(segments.size());

  // Add each segment
  for(const std::vector<vtkIdType>& line : segments)
  {
    noether::ToolPathSegment segment;
    segment.reserve(line.size());

    for(std::size_t i = 0; i < line.size(); ++i)
    {
      const vtkIdType& this_id = line[i];
      const Eigen::Vector3d point = Eigen::Map<const Eigen::Vector3d>(points->GetPoint(this_id));
      const Eigen::Vector3d normal = Eigen::Map<const Eigen::Vector3d>(point_data->GetNormals()->GetTuple(this_id));

      Eigen::Vector3d path_dir = Eigen::Vector3d::UnitX();
      if (i < line.size() - 1)
      {
        const vtkIdType& next_id = line[i + 1];
        const Eigen::Vector3d next_point = Eigen::Map<const Eigen::Vector3d>(points->GetPoint(next_id));
        path_dir = next_point - point;
      }
      else
      {
        const vtkIdType& prev_id = line[i - 1];
        const Eigen::Vector3d prev_point = Eigen::Map<const Eigen::Vector3d>(points->GetPoint(prev_id));
        path_dir = point - prev_point;
      }

      Eigen::Isometry3d pose = createTransform(point, normal, path_dir);
      segment.push_back(pose);
    }

    tool_path.push_back(segment);
  }

  return tool_path;
}

/**
 * @brief Gets the distances (normal to the cut plane) from the cut origin to the closest and furthest corners of the
 * mesh
 * @param obb_size Column-wise matrix of the object-aligned size vectors of the mesh (from PCA), relative to the mesh
 * origin
 * @param pca_centroid Centroid of the mesh determined by PCA, relative to the mesh origin
 * @param origin Origin of the raster pattern, relative to the mesh origin
 * @param dir Raster cut plane normal, relative to the mesh origin
 * @return Tuple of (min distance, max distance)
 */
static std::tuple<double, double> getDistancesToMinMaxCuts(const Eigen::Matrix3d& obb_size,
                                                           const Eigen::Vector3d& obb_centroid,
                                                           const Eigen::Vector3d& cut_origin,
                                                           const Eigen::Vector3d& cut_normal)
{
  /* Create the 8 corners of the object-aligned bounding box using vectorization
   *
   * | x0  x1  x2 | * | 1 -1  1 -1  1 -1  1 -1 | +  | ox | = | cx0  cx1  ... |
   * | y0  y1  y2 |   | 1  1 -1 -1  1  1 -1 -1 |    | oy |   | cy0  cy1  ... |
   * | z0  z1  zz |   | 1  1  1  1 -1 -1 -1 -1 |    | oz |   | cz0  cz1  ... |
   */
  Eigen::MatrixXi corner_mask(3, 8);
  // clang-format off
  corner_mask <<
      1, -1, 1, -1, 1, -1, 1, -1,
      1, 1, -1, -1, 1, 1, -1, -1,
      1, 1, 1, 1, -1, -1, -1, -1;
  // clang-format on
  Eigen::MatrixXd corners = (obb_size / 2.0) * corner_mask.cast<double>();
  corners.colwise() += obb_centroid;

  // For each corner, compute the distance from the raster origin to a plane that passes through the OBB corner and
  // whose normal is the input raster direction. Save the largest and smallest distances to the corners
  double d_max = -std::numeric_limits<double>::max();
  double d_min = std::numeric_limits<double>::max();
  for (Eigen::Index col = 0; col < corners.cols(); ++col)
  {
    Eigen::Hyperplane<double, 3> plane(cut_normal, corners.col(col));
    double d = plane.signedDistance(cut_origin);

    // Negate the signed distance because points "in front" of the cut plane have to travel in the negative plane
    // direction to get back to the plane
    d *= -1;

    if (d > d_max)
      d_max = d;
    else if (d < d_min)
      d_min = d;
  }

  return std::make_tuple(d_min, d_max);
}

}  // namespace

namespace noether
{
PlaneSlicerRasterPlanner::PlaneSlicerRasterPlanner(DirectionGenerator::ConstPtr dir_gen,
                                                   OriginGenerator::ConstPtr origin_gen)
  : RasterPlanner(std::move(dir_gen), std::move(origin_gen))
{
}

void PlaneSlicerRasterPlanner::setMinSegmentSize(const double min_segment_size)
{
  min_segment_size_ = min_segment_size;
}

void PlaneSlicerRasterPlanner::generateRastersBidirectionally(const bool bidirectional)
{
  bidirectional_ = bidirectional;
}

ToolPaths PlaneSlicerRasterPlanner::planImpl(const pcl::PolygonMesh& mesh) const
{
  if (!hasNormals(mesh.cloud))
  {
    std::stringstream ss;
    ss << "The input mesh does not have vertex normals, which are required for the plane slice raster tool path "
          "planner. "
       << "Use a MeshModifier to generate vertex normals for the mesh, or provide a different mesh with vertex "
          "normals.";
    throw std::runtime_error(ss.str());
  }

  // Use principal component analysis (PCA) to determine the principal axes of the mesh
  Eigen::Vector3d mesh_normal;  // Unit vector along shortest mesh PCA direction
  Eigen::Matrix3d pca_vecs;     // Principal axes, scaled to the size of the mesh in each direction
  Eigen::Vector3d centroid;     // Mesh centroid
  {
    // Use Original PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    // Perform PCA analysis
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);

    // Get the extents of the point cloud relative to its mean and principal axes
    pcl::PointCloud<pcl::PointXYZ> proj;
    pca.project(*cloud, proj);
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(proj, min, max);
    Eigen::Array3f scales = max.getArray3fMap() - min.getArray3fMap();

    mesh_normal = pca.getEigenVectors().col(2).cast<double>().normalized();
    centroid = pca.getMean().head<3>().cast<double>();
    pca_vecs = (pca.getEigenVectors().array().rowwise() * scales.transpose()).cast<double>();
  }

  // Get the initial cutting plane
  const Eigen::Vector3d cut_direction = dir_gen_->generate(mesh);
  const Eigen::Vector3d cut_normal = (cut_direction.normalized().cross(mesh_normal)).normalized();

  // Get the initial plane starting location
  const Eigen::Vector3d cut_origin = origin_gen_->generate(mesh);

  double d_min_cut, d_max_cut;
  std::tie(d_min_cut, d_max_cut) = getDistancesToMinMaxCuts(pca_vecs, centroid, cut_origin, cut_normal);

  // If we don't want to generate rasters bidirectionally...
  if (!bidirectional_)
  {
    // ... and the furthest cut distance is behind the cutting plane, then don't make any cuts
    if (d_max_cut < 0.0)
      return {};

    // ... and the closest cut distance is behined the cutting plane, set the distance to the closest cutting plane
    // equal to zero (i.e., at the nominal cut origin)
    if (d_min_cut < 0.0)
      d_min_cut = 0.0;
  }

  const double cut_span = std::abs(d_max_cut - d_min_cut);
  const auto num_planes = static_cast<std::size_t>(std::ceil(cut_span / line_spacing_));

  // Compute the starting raster location such that one cut passes through the cut origin
  // The start location needs to move some integer number of line spacings (determined by the distance to the minimum
  // cut) in the cut normal direction
  const auto n_planes_start_offset = static_cast<std::size_t>(std::floor(std::abs(d_min_cut) / line_spacing_));
  const double start_offset = n_planes_start_offset * line_spacing_;
  const Eigen::Vector3d start_loc = cut_origin + cut_normal * std::copysign(start_offset, d_min_cut);

  // Convert input mesh to VTK type & calculate normals if necessary
  vtkSmartPointer<vtkPolyData> mesh_data = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(mesh, mesh_data);

  ToolPaths tool_paths;
  tool_paths.reserve(num_planes);

  // Create and run a VTK cutter+stripper pipeline for each plane
  // Note: the pipeline is created and run per-plane in order to be able to differentiate between segments on
  for (std::size_t i = 0; i < num_planes; ++i)
  {
    const Eigen::Vector3d current_loc = start_loc + i * line_spacing_ * cut_normal;

    vtkNew<vtkPlane> plane;
    plane->SetOrigin(current_loc.x(), current_loc.y(), current_loc.z());
    plane->SetNormal(cut_normal.x(), cut_normal.y(), cut_normal.z());

    vtkNew<vtkCutter> cutter;
    cutter->SetCutFunction(plane);
    cutter->SetSortByToSortByCell();
    cutter->SetInputData(mesh_data);
    cutter->Update();

    vtkNew<vtkStripper> stripper;
    // Disable contiguous segment joining because it does not ensure correct segment ordering before joining
    stripper->JoinContiguousSegmentsOff();
    stripper->SetMaximumLength(mesh_data->GetNumberOfPoints());
    stripper->SetInputConnection(cutter->GetOutputPort());
    stripper->Update();

    if (stripper->GetErrorCode() != vtkErrorCode::NoError)
      continue;

    auto tool_path = convertToPoses(stripper->GetOutput(), cut_direction, min_hole_size_);
    if (!tool_path.empty())
      tool_paths.push_back(tool_path);
  }

  return tool_paths;
}

}  // namespace noether
