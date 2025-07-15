#include <noether_tpp/tool_path_planners/raster/origin_generators/centroid_origin_generator.h>

#include <pcl/common/centroid.h>
#include <yaml-cpp/yaml.h>

namespace noether
{
Eigen::Vector3d CentroidOriginGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);
  pcl::PointXYZ centroid;
  if (pcl::computeCentroid(vertices, centroid) < 1)
    throw std::runtime_error("Failed to compute mesh centroid");

  return centroid.getArray3fMap().matrix().cast<double>();
}

}  // namespace noether

namespace YAML
{
Node convert<noether::CentroidOriginGenerator>::encode(const T& val) { return {}; }

bool convert<noether::CentroidOriginGenerator>::decode(const Node& node, T& val) { return true; }

}  // namespace YAML
