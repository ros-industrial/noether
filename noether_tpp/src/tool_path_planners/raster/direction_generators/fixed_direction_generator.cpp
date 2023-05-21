#include <noether_tpp/tool_path_planners/raster/direction_generators/fixed_direction_generator.h>

namespace noether
{
FixedDirectionGenerator::FixedDirectionGenerator(const Eigen::Vector3d& direction) : direction_(direction.normalized())
{
}

Eigen::Vector3d FixedDirectionGenerator::generate(const pcl::PolygonMesh&) const { return direction_; }

}  // namespace noether
