#include <noether_tpp/core/tool_path_modifier.h>

#include <pcl/PCLPointField.h>
#include <pcl/geometry/mesh_traits.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/PolygonMesh.h>

namespace noether
{
/** @brief Estimates the direction of travel of a tool path by averaging the vectors between adjacent waypoints */
Eigen::Vector3d estimateToolPathDirection(const ToolPath& tool_path);

/**
 * @brief Estimates the raster direction of a set of tool paths by returning the component of the vector from the first
 * point in the first segment of the first tool path) to the first waypoint in the first segment of the last tool path
 * that is perpendicular to the nominal tool path direction
 */
Eigen::Vector3d estimateRasterDirection(const ToolPaths& tool_paths, const Eigen::Vector3d& reference_tool_path_dir);

std::vector<pcl::PCLPointField>::const_iterator findField(const std::vector<pcl::PCLPointField>& fields,
                                                          const std::string& name);

std::vector<pcl::PCLPointField>::const_iterator findFieldOrThrow(const std::vector<pcl::PCLPointField>& fields,
                                                                 const std::string& name);

bool hasNormals(const pcl::PolygonMesh& mesh);

Eigen::Vector3f getPoint(const pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx);

Eigen::Vector3f getNormal(const pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx);

using MeshTraits = pcl::geometry::DefaultMeshTraits<std::uint32_t>;
using TriangleMesh = pcl::geometry::TriangleMesh<MeshTraits>;

/**
 * @brief Creates a triangle mesh representation of the input polygon mesh
 */
TriangleMesh createTriangleMesh(const pcl::PolygonMesh& input);

/**
 * @brief Computes the length of a tool path segment
 */
std::tuple<double, std::vector<double>> computeLength(const ToolPathSegment& segment);

}  // namespace noether
