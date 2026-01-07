#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
#include <noether_tpp/utils.h>
#include <noether_tpp/tool_path_modifiers/uniform_spacing_linear_modifier.h>

#include <boost/graph/directed_graph.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <stdexcept>
#include <stack>
#include <utility>
#include <vector>
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
/**
 * @brief Finds the single shared vertex ID between two line possibly connected segments
 * @return The ID of the single shared vertex (if one exists); otherwise returns a null optional
 * @throws Throws an exception if the segments share more than one vertex
 */
std::optional<vtkIdType> findConnectingVertex(vtkSmartPointer<vtkIdList> s1, vtkSmartPointer<vtkIdList> s2)
{
  vtkNew<vtkIdList> intersection;
  intersection->DeepCopy(s1);
  intersection->IntersectWith(s2);

  if (intersection->GetNumberOfIds() > 1)
    throw std::runtime_error("Too many connecting vertices found between adjacent segments");

  if (intersection->GetNumberOfIds() == 0)
    return {};

  return intersection->GetId(0);
}

/** @details Typedef for a vertex in a graph of line segments, where the first element is the segment index to which the
 * cut vertex belongs and the second is ID of the cut vertex itself */
using Vertex = std::pair<std::size_t, vtkIdType>;
/** @details Typedef for a graph of line segments */
using Graph = boost::directed_graph<Vertex>;
/** @details Vertex for a vertex descriptor */
using VertexDescriptor = boost::graph_traits<Graph>::vertex_descriptor;

/**
 * @details Finds the vertex descriptor in the graph that matches the input vertex information
 * @param g Graph
 * @param vertex Vertex (i.e., segment index and cut vertex ID)
 * @return
 */
VertexDescriptor findVertexDescriptor(const Graph& g, const Vertex& vertex)
{
  Graph::vertex_iterator it, end;
  for (boost::tie(it, end) = boost::vertices(g); it != end; ++it)
  {
    if (g[*it] == vertex)
      return *it;
  }

  return Graph::null_vertex();
}

/**
 * @brief Performs a topological sort on the line segments in the input polydata
 * @details The output from a VTKCutter + VTKStripper is a set of potentially unorganized line segments, which may or
 * may not have shared vertices between them. In order to easily join contiguous segments (those which have a shared
 * vertex), we first need to order the segments such that physically adjacent segments (i.e., contiguous segments with a
 * shared vertex) are also adjacent in the data container that holds them (i.e., the VTKCellArray). Under the following
 * assumptions, we can formulate this task as as a topological sort of a directed acyclic graph:
 *   1. The line segments are inherently directed graphs (e.g., no cycles)
 *   2. Each line segment has at most 2 connections to other line segments (e.g. at the front and back of the segment)
 *   3. The line segments together do not form a cycle
 * For segments that share a connecting vertex, we add an edge in the graph.
 * The direction of this edge can be arbitrary because we only care about sorting the order of the segments in the
 * container. A different function can be used later to order and join contiguous segments.
 * @param polydata VTK polydata output from a VTKStripper (run after a VTKCutter)
 */
void topologicalSortSegments(vtkSmartPointer<vtkPolyData> polydata)
{
  // Get the line segments from the polydata
  vtkSmartPointer<vtkCellArray> segments = polydata->GetLines();

  const std::size_t n_segments = segments->GetNumberOfCells();
  if (n_segments <= 1)
    return;

  // Create the graph
  Graph graph;

  // Add the nodes (i.e., vertices in the segments) and the edge between them and the adjacent nodes
  for (std::size_t i = 0; i < n_segments; ++i)
  {
    // Extract segment i
    vtkNew<vtkIdList> point_ids;
    segments->GetCellAtId(i, point_ids);
    const std::size_t n_vertices = point_ids->GetNumberOfIds();

    // Add a vertex for the 0th point
    VertexDescriptor v_prev = boost::add_vertex(std::make_pair(i, point_ids->GetId(0)), graph);

    // Add vertices for the rest of the vertices
    for (std::size_t j = 1; j < n_vertices; ++j)
    {
      VertexDescriptor v_next = boost::add_vertex(std::make_pair(i, point_ids->GetId(j)), graph);

      // Add an edge between adjacent vertices
      boost::add_edge(v_prev, v_next, graph);

      v_prev = v_next;
    }
  }

  // Check all possible combinations of segments for a connecting vertex and add an edge for each connection found
  for (std::size_t i = 0; i < n_segments - 1; ++i)
  {
    // Get the first segment
    vtkNew<vtkIdList> segment_i;
    segments->GetCellAtId(i, segment_i);

    // Get the next possible segment
    for (std::size_t j = i + 1; j < n_segments; ++j)
    {
      vtkNew<vtkIdList> segment_j;
      segments->GetCellAtId(j, segment_j);

      // Find the intersection between the segments and add an edge (of arbitrary direction) between them
      std::optional<vtkIdType> intersection = findConnectingVertex(segment_i, segment_j);
      if (intersection)
      {
        VertexDescriptor v1 = findVertexDescriptor(graph, std::make_pair(i, *intersection));
        VertexDescriptor v2 = findVertexDescriptor(graph, std::make_pair(j, *intersection));
        boost::add_edge(v1, v2, graph);
      }
    }
  }

  // Run the topological sort
  std::vector<VertexDescriptor> topo_sort;
  boost::topological_sort(graph, std::back_inserter(topo_sort));
  std::reverse(topo_sort.begin(), topo_sort.end());

  // Reduce the result of the topological sort to the sorted segment indices
  std::vector<std::size_t> sorted_idx;
  for (const VertexDescriptor& v : topo_sort)
    if (std::find(sorted_idx.begin(), sorted_idx.end(), graph[v].first) == sorted_idx.end())
      sorted_idx.push_back(graph[v].first);

  // Check that the output has the same number of segments as the input
  if (sorted_idx.size() != n_segments)
    throw std::runtime_error("Topological sort failed");

  // Use the sorted segment indices to create a new ordered container of segments
  vtkNew<vtkCellArray> sorted_segments;
  for (const std::size_t idx : sorted_idx)
  {
    vtkNew<vtkIdList> point_ids;
    segments->GetCellAtId(idx, point_ids);
    sorted_segments->InsertNextCell(point_ids);
  }

  // Overwrite the lines in the polydata with the sorted segments
  polydata->SetLines(sorted_segments);
}

/**
 * @brief Unifies the direction of all line segments
 * @details By default, VTKCutter and VTKStripper do not ensure that all created segments "point" the same way.
 * This function iterates through all segments and compares their direction (computed as the vector between the first
 * two points) with a reference direction (e.g., the raster cut direction). If the dot product of these two vectors is
 * less than 0, the order of the vertices in the line segment is reversed.
 * @param polydata VTK Polydata output from a VTKCutter + VTKStripper
 * @param reference_direction
 */
void unifySegmentDirection(vtkSmartPointer<vtkPolyData> polydata, const Eigen::Vector3d& reference_direction)
{
  vtkSmartPointer<vtkPoints> points = polydata->GetPoints();
  vtkSmartPointer<vtkCellArray> segments = polydata->GetLines();

  for (std::size_t i = 0; i < segments->GetNumberOfCells(); ++i)
  {
    vtkNew<vtkIdList> segment;
    segments->GetCellAtId(i, segment);

    // Compute the direction vector between the first and second points in the segment
    const Eigen::Vector3d p1 = Eigen::Map<const Eigen::Vector3d>(points->GetPoint(segment->GetId(0)));
    const Eigen::Vector3d p2 = Eigen::Map<const Eigen::Vector3d>(points->GetPoint(segment->GetId(1)));
    const Eigen::Vector3d dir = p2 - p1;

    // Reverse the order of the segment if it does not align with the reference direction
    if (dir.dot(reference_direction) < 0.0)
    {
      segments->ReverseCellAtId(i);
    }
  }
}

/**
 * @brief Concatenates the unique entries in two line segments into a new line segment
 */
vtkSmartPointer<vtkIdList> concatenateUnique(vtkSmartPointer<vtkIdList> l1, vtkSmartPointer<vtkIdList> l2)
{
  vtkNew<vtkIdList> output;
  output->DeepCopy(l1);
  const std::size_t l = output->GetNumberOfIds();
  for (std::size_t i = 0; i < l2->GetNumberOfIds(); ++i)
  {
    std::size_t idx = output->InsertUniqueId(l2->GetId(i));
    std::size_t foo = idx * 1;
  }
  return output;
}

/**
 * @brief Creates a reversed copy of the input line segment
 */
vtkSmartPointer<vtkIdList> reverse(vtkSmartPointer<vtkIdList> l)
{
  vtkNew<vtkIdList> output;
  for (std::size_t i = l->GetNumberOfIds(); i-- > 0;)
    output->InsertNextId(l->GetId(i));
  return output;
}

/**
 * @brief Joins two segments that share a connecting vertex
 * @details We can only (easily) handle 4 cases for the location of the connecting vertex:
 *   1. End of segment 1, beginning of segment 2 -> add a new segment to the stack that is [s1, s2]
 *   2. Beginning of segment 1, end of segment 2 -> add a new segment to the stack that is [s2, s1]
 *   3. Beginning of segment 1, beginning of segment 2 -> add a new segment to the stack that is [reverse(s2), s1]
 *   4. End of segment 1, end of segment 2 -> add a new segment to the stack that is [s1, reverse(s2)]
 * @note When joining segments into a new segment, we purposefully do not duplicate the shared vertex
 */
vtkSmartPointer<vtkIdList> joinContiguousSegments(vtkSmartPointer<vtkIdList> s1,
                                                  vtkSmartPointer<vtkIdList> s2,
                                                  const vtkIdType connecting_vertex)
{
  // Find the iterator to the connection vertex in each segment
  std::size_t idx_s1 = s1->FindIdLocation(connecting_vertex);
  std::size_t idx_s2 = s2->FindIdLocation(connecting_vertex);

  if (idx_s1 == 0)
  {
    if (idx_s2 == s2->GetNumberOfIds() - 1)
    {
      // Connection occurs at beginning of segment 1 and end of segment 2 -> add [s2, s1]
      return concatenateUnique(s2, s1);
    }
    else if (idx_s2 == 0)
    {
      // Connection occurs at beginning of segment 1 and beginning of segment 2 -> add [reverse(s2), s1]
      return concatenateUnique(reverse(s2), s1);
    }
    else
    {
      throw std::runtime_error("Connecting vertex exists at the beginning of one segment but not at the end or "
                               "beginning of the adjacent segment");
    }
  }
  else if (idx_s1 == s1->GetNumberOfIds() - 1)
  {
    if (idx_s2 == 0)
    {
      // Connection occurs at end of segment 1 and beginning of segment 2 -> add [s1, s2]
      return concatenateUnique(s1, s2);
    }
    else if (idx_s2 == s2->GetNumberOfIds() - 1)
    {
      // Connection occurs at end of segment 1 and end of segment 2 -> add [s1, reverse(s2)]
      return concatenateUnique(s1, reverse(s2));
    }
    else
    {
      throw std::runtime_error("Connecting vertex exists at the end of one segment but not at the beginning or end "
                               "of the adjacent segment");
    }
  }

  throw std::runtime_error("Connecting vertex does not exist at the start or end of a segment");
}

/**
 * @brief Joins contiguous line segments (i.e., those which are connected by a shared vertex)
 * @details The method `JoinContiguousSegmentsOn` in VTKStripper does not ensure that contiguous segments are joined in
 * the correct order. This function finds shared vertices between adjacent line segments and iteratively reorders them
 * correctly and joins them together under the following assumptions:
 *   * Contiguity is denoted by the line segments containing a shared vertex ID
 *   * The input line segments are topologically sorted such that two segments that are adjacent in the data container
 * could potentially have a shared vertex.
 * @param polydata VTK Polydata output from a VTKCutter/VTKStripper pipeline that has been processed by the
 * `topologicalSortSegments` and `unifySegmentDirection` functions
 * @note This function should run after `topologicalSortSegments` and `unifySegmentDirection`
 */
void joinContiguousSegments(vtkSmartPointer<vtkPolyData> polydata)
{
  vtkSmartPointer<vtkCellArray> segments = polydata->GetLines();

  const std::size_t n_segments = segments->GetNumberOfCells();
  if (n_segments <= 1)
    return;

  // Initialize the output
  vtkNew<vtkCellArray> output;

  // Create a stack from the input segments (in reverse order, such that the first segment is at the top of the stack)
  std::stack<vtkSmartPointer<vtkIdList>> stack;
  for (std::size_t i = n_segments; i-- > 0;)
  {
    vtkNew<vtkIdList> point_ids;
    segments->GetCellAtId(i, point_ids);
    stack.push(point_ids);
  }

  while (!stack.empty())
  {
    // Get the first segment
    vtkSmartPointer<vtkIdList> s1 = stack.top();
    stack.pop();

    // Check if stack is empty
    if (stack.empty())
    {
      output->InsertNextCell(s1);
      continue;
    }

    // Get the second segment
    vtkSmartPointer<vtkIdList> s2 = stack.top();
    stack.pop();

    // Find the connecting vertex ID
    std::optional<vtkIdType> connecting_vertex = findConnectingVertex(s1, s2);

    // If no connection exists between the two segments, add the first segment to the output and return the second
    // segment to the stack
    if (!connecting_vertex)
    {
      output->InsertNextCell(s1);
      stack.push(s2);
    }
    else
    {
      // Otherwise, join the contiguous segments and add the resulting segment to the stack
      stack.push(joinContiguousSegments(s1, s2, *connecting_vertex));
    }
  }

  // Overwrite the lines in the polydata with the results
  polydata->SetLines(output);
}

/**
 * @brief Processes the polylines in the output of a VTKCutter/VTKStripper pipeline such that contiguous segments are
 * joined in the correct order, all segments have a consistent direction, and close segments are joined together.
 * @param polydata VTK Polydata output from a VTKCutter/VTKStripper pipeline
 * @param reference_direction Reference segemnt direction (e.g., raster cut direction)
 * @param max_segment_separation Distance (m) below which adjacent unconnected segments should be joined
 */
void processPolyLines(vtkSmartPointer<vtkPolyData> polydata,
                      const Eigen::Vector3d& reference_direction,
                      const double max_segment_separation)
{
  // Ensure that all of the segments are pointing the same direction (relative to the reference direction)
  unifySegmentDirection(polydata, reference_direction);

  // Topologically sort the segments such that connected segments (e.g., ones that share a common vertex) will be
  // adjacent in the the container
  topologicalSortSegments(polydata);

  // Join the contiguous segments
  joinContiguousSegments(polydata);
}

Eigen::Isometry3d createTransform(const Eigen::Vector3d& position,
                                  const Eigen::Vector3d& normal,
                                  const Eigen::Vector3d& path_dir)
{
  Eigen::Vector3d y = normal.cross(path_dir);
  Eigen::Vector3d x = y.cross(normal);

  Eigen::Isometry3d transform;
  transform.matrix().col(0).head<3>() = x.normalized();
  transform.matrix().col(1).head<3>() = y.normalized();
  transform.matrix().col(2).head<3>() = normal.normalized();
  transform.matrix().col(3).head<3>() = position;

  return transform;
}

noether::ToolPath convertToPoses(vtkSmartPointer<vtkPolyData> polydata)
{
  // Extract the point and normal data from the input polydata
  vtkSmartPointer<vtkCellArray> segments = polydata->GetLines();
  vtkSmartPointer<vtkPoints> points = polydata->GetPoints();
  vtkSmartPointer<vtkPointData> point_data = polydata->GetPointData();

  const std::size_t n_segments = segments->GetNumberOfCells();

  // Create the output container
  noether::ToolPath tool_path;
  tool_path.reserve(n_segments);

  // Add each segment
  for (std::size_t s_idx = 0; s_idx < n_segments; ++s_idx)
  {
    vtkNew<vtkIdList> segment_vtk;
    segments->GetCellAtId(s_idx, segment_vtk);
    const std::size_t n_points = segment_vtk->GetNumberOfIds();

    noether::ToolPathSegment segment;
    segment.reserve(n_points);

    for (std::size_t pt_idx = 0; pt_idx < n_points; ++pt_idx)
    {
      vtkIdType this_id = segment_vtk->GetId(pt_idx);
      const Eigen::Vector3d point = Eigen::Map<const Eigen::Vector3d>(points->GetPoint(this_id));
      const Eigen::Vector3d normal = Eigen::Map<const Eigen::Vector3d>(point_data->GetNormals()->GetTuple(this_id));

      // Compute the path direction with either the next or previous point
      Eigen::Vector3d path_dir;
      if (pt_idx < n_points - 1)
      {
        const vtkIdType next_id = segment_vtk->GetId(pt_idx + 1);
        const Eigen::Vector3d next_point = Eigen::Map<const Eigen::Vector3d>(points->GetPoint(next_id));
        path_dir = next_point - point;
      }
      else
      {
        const vtkIdType prev_id = segment_vtk->GetId(pt_idx - 1);
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

void PlaneSlicerRasterPlanner::setPointSpacing(const double point_spacing) { point_spacing_ = point_spacing; }

void PlaneSlicerRasterPlanner::setMinHoleSize(const double min_hole_size) { min_hole_size_ = min_hole_size; };

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

  // Create and run a VTK cutter + stripper pipeline for each plane
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

    // Process the tool path lines to ensure proper segment ordering, direction, and connectivity
    vtkNew<vtkPolyData> processed_output;
    processed_output->DeepCopy(stripper->GetOutput());
    processPolyLines(processed_output, cut_direction, min_hole_size_);

    auto tool_path = convertToPoses(processed_output);
    if (!tool_path.empty())
      tool_paths.push_back(tool_path);
  }

  // Add a linear uniform sampling tool path modifier to ensure the point spacing requirement is met
  UniformSpacingLinearModifier modifier(point_spacing_);
  return modifier.modify(tool_paths);
}

}  // namespace noether
