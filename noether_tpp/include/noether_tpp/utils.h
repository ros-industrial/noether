#include <noether_tpp/core/tool_path_modifier.h>
#include <pcl/PCLPointField.h>

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

}  // namespace noether
