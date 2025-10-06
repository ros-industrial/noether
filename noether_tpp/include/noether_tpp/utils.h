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

Eigen::Vector3f getFaceNormal(const pcl::PolygonMesh& mesh, const pcl::Vertices& polygon);

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

/**
 * @brief Unrolls an exception and captures content from all nested exceptions
 * @details Adapted from https://en.cppreference.com/w/cpp/error/throw_with_nested
 */
void printException(const std::exception& e, std::ostream& ss, int level = 0);

/* @brief Creates a mesh of an xy plane with a specified length and width.
 * @details The plane normal is the z-axis and the centroid of the plane located at the specified origin.
 *
 * The plane mesh has the following structure. 0-3 represent vertices and A and B represent polygons.
 *
 *    0---3
 *    |A /|
 *    |/ B|
 *    1---2
 *
 * @param lx Length (m) along the x direction
 * @param ly Length (m) along the y direction
 */
pcl::PolygonMesh createPlaneMesh(const float lx = 1.0,
                                 const float ly = 1.0,
                                 const Eigen::Isometry3d& origin = Eigen::Isometry3d::Identity());

/* @brief Creates a mesh of an ellipsoid with specified semi-axes and resolution.
 * @details The centroid of the ellipsoid is the specifed origin. The implementation is adapted from Open3d
 * https://github.com/isl-org/Open3D/blob/v0.19.0/cpp/open3d/geometry/TriangleMeshFactory.cpp#L216-L381
 *
 * Equation of an ellipsoid is x^2/a^2 + y^2/b^2 + z^2/c^2 = 1
 * where a, b, c are the semi-axes in meters
 *
 * Parameterization of ellipsoid from spherical coordinates to cartesian coordinates where 0 <= theta <= PI and 0 <= phi
 * <= 2*PI x = a * sin(theta) * cos(phi) y = b * sin(theta) * sin(phi) z = c * cos(theta)
 *
 * @param rx Radius in the x direction
 * @param ry Radius in the y direction
 * @param rz Radius in the z direction
 *
 */
pcl::PolygonMesh createEllipsoidMesh(const float rx = 2.0,
                                     const float ry = 2.0,
                                     const float rz = 1.0,
                                     const int resolution = 20,
                                     const Eigen::Isometry3d& origin = Eigen::Isometry3d::Identity());

}  // namespace noether
