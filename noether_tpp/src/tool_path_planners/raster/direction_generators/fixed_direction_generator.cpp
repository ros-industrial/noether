#include <noether_tpp/tool_path_planners/raster/direction_generators/fixed_direction_generator.h>
#include <noether_tpp/serialization.h>

namespace noether
{
FixedDirectionGenerator::FixedDirectionGenerator(const Eigen::Vector3d& direction) : direction_(direction.normalized())
{
}

Eigen::Vector3d FixedDirectionGenerator::generate(const pcl::PolygonMesh&) const { return direction_; }

}  // namespace noether

namespace YAML
{
Node convert<noether::FixedDirectionGenerator>::encode(const T& val)
{
  Node node;
  node["direction"] = val.direction_;
  return node;
}

bool convert<noether::FixedDirectionGenerator>::decode(const Node& node, T& val)
{
  val.direction_ = getMember<Eigen::Vector3d>(node, "direction");
  return true;
}

}  // namespace YAML
