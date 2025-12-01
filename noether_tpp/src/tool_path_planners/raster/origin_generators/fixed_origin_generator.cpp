#include <noether_tpp/tool_path_planners/raster/origin_generators/fixed_origin_generator.h>
#include <noether_tpp/serialization.h>

namespace noether
{
FixedOriginGenerator::FixedOriginGenerator(const Eigen::Vector3d& origin) : origin_(origin) {}

Eigen::Vector3d FixedOriginGenerator::generate(const pcl::PolygonMesh& mesh) const { return origin_; }

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::FixedOriginGenerator>::encode(const noether::FixedOriginGenerator& val)
{
  Node node;
  node["origin"] = val.origin_;
  return node;
}

bool convert<noether::FixedOriginGenerator>::decode(const Node& node, noether::FixedOriginGenerator& val)
{
  val.origin_ = getMember<Eigen::Vector3d>(node, "origin");
  return true;
}
/** @endcond */

}  // namespace YAML
