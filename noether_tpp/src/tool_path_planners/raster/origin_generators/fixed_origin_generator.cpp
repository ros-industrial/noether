#include <noether_tpp/tool_path_planners/raster/origin_generators/fixed_origin_generator.h>

namespace noether
{
FixedOriginGenerator::FixedOriginGenerator(const Eigen::Vector3d& origin) : origin_(origin) {}

Eigen::Vector3d FixedOriginGenerator::generate(const pcl::PolygonMesh& mesh) const { return origin_; }

}  // namespace noether
