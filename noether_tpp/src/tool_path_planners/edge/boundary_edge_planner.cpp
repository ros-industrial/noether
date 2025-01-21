#include <noether_tpp/tool_path_planners/edge/boundary_edge_planner.h>
#include <noether_tpp/utils.h>

namespace
{
/**
 * @brief Returns a collection of half-edges of the mesh that are on its boundaries
 * @sa https://github.com/PointCloudLibrary/pcl/blob/pcl-1.10.0/examples/geometry/example_half_edge_mesh.cpp#L188-L199
 */
template <class MeshT>
std::vector<typename MeshT::HalfEdgeIndices> getBoundaryHalfEdges(const MeshT& mesh, const size_t expected_size = 3)
{
  typedef MeshT Mesh;
  typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
  typedef typename Mesh::HalfEdgeIndices HalfEdgeIndices;
  typedef typename Mesh::InnerHalfEdgeAroundFaceCirculator IHEAFC;

  std::vector<typename MeshT::HalfEdgeIndices> boundary_he_collection;

  HalfEdgeIndices boundary_he;
  boundary_he.reserve(expected_size);
  std::vector<bool> visited(mesh.sizeEdges(), false);

  for (HalfEdgeIndex i(0); i < HalfEdgeIndex(mesh.sizeHalfEdges()); ++i)
  {
    if (mesh.isBoundary(i) && !visited[pcl::geometry::toEdgeIndex(i).get()])
    {
      boundary_he.clear();

      IHEAFC circ = mesh.getInnerHalfEdgeAroundFaceCirculator(i);
      IHEAFC circ_end = circ;
      do
      {
        HalfEdgeIndex he_idx = circ.getTargetIndex();
        if (!he_idx.isValid())
          break;

        visited[pcl::geometry::toEdgeIndex(he_idx).get()] = true;
        boundary_he.push_back(circ.getTargetIndex());
      } while (++circ != circ_end);

      boundary_he_collection.push_back(boundary_he);
    }
  }

  return boundary_he_collection;
}

Eigen::Matrix3f setOrientation(const Eigen::Vector3f& z_axis, const Eigen::Vector3f& x_dir)
{
  // The y-axis equals the z-axis cross the x-direction
  const Eigen::Vector3f y_axis = z_axis.cross(x_dir);

  // The x-axis equals the y-axis cross the z-axis
  const Eigen::Vector3f x_axis = y_axis.cross(z_axis);

  Eigen::Matrix3f mat;
  mat.col(0) = x_axis;
  mat.col(1) = y_axis;
  mat.col(2) = z_axis;

  return mat;
}

}  // namespace

namespace noether
{
BoundaryEdgePlanner::BoundaryEdgePlanner() : EdgePlanner() {}

ToolPaths BoundaryEdgePlanner::planImpl(const pcl::PolygonMesh& mesh) const
{
  // Create a triangle mesh representation of the input mesh, compatible with half-edge calcuation
  TriangleMesh tri_mesh = createTriangleMesh(mesh);

  // Identify the indices of the half-edge edges in the mesh
  std::vector<TriangleMesh::HalfEdgeIndices> boundaries = getBoundaryHalfEdges(tri_mesh);

  ToolPaths tool_paths;
  tool_paths.reserve(boundaries.size());

  // Traverse each boundary and convert to a tool path
  for (auto boundary = boundaries.begin(); boundary != boundaries.end(); ++boundary)
  {
    ToolPathSegment segment;
    segment.reserve(boundary->size());

    for (auto edge = boundary->begin(); edge != boundary->end(); ++edge)
    {
      // Get the source vertex of the edge
      const TriangleMesh::VertexIndex source = tri_mesh.getOriginatingVertexIndex(*edge);
      const std::uint32_t source_idx = tri_mesh.getVertexDataCloud()[source.get()];
      const Eigen::Vector3f source_point = getPoint(mesh.cloud, source_idx);
      const Eigen::Vector3f source_normal = getNormal(mesh.cloud, source_idx);

      // Get the target vertex of the edge
      const TriangleMesh::VertexIndex target = tri_mesh.getTerminatingVertexIndex(*edge);
      const std::uint32_t target_idx = tri_mesh.getVertexDataCloud()[target.get()];
      const Eigen::Vector3f target_point = getPoint(mesh.cloud, target_idx);

      // Generate the pose of the tool path waypoint
      Eigen::Isometry3f pose;

      // The position of the waypoint equals the position of the source vertex
      pose.translation() = source_point;

      // The z-axis of the pose equals the source point normal
      // The x-axis aligns with the vector from the source vertex to the target vertex
      const Eigen::Vector3f x_dir = (target_point - source_point).normalized();
      pose.linear() = setOrientation(source_normal, x_dir);

      // Add the pose to the tool path segment
      segment.push_back(pose.cast<double>());
    }

    // Add the segment into a single tool path into the tool paths output object
    tool_paths.push_back(ToolPath({ segment }));
  }

  return tool_paths;
}

ToolPathPlanner::ConstPtr BoundaryEdgePlannerFactory::create() const { return std::make_unique<BoundaryEdgePlanner>(); }

}  // namespace noether
