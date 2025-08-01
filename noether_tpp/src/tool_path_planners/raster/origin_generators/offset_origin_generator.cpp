#include <noether_tpp/tool_path_planners/raster/origin_generators/offset_origin_generator.h>

namespace noether
{
OffsetOriginGenerator::OffsetOriginGenerator(OriginGenerator::ConstPtr&& generator, const Eigen::Vector3d& offset)
  : generator_(std::move(generator)), offset_(offset)
{
}

Eigen::Vector3d OffsetOriginGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  Eigen::Vector3d origin = generator_->generate(mesh);
  return origin + offset_;
}

}  // namespace noether
