#include <noether_tpp/mesh_modifiers/compound_modifier.h>

namespace noether
{
CompoundMeshModifier::CompoundMeshModifier(std::vector<MeshModifier::ConstPtr> modifiers)
  : modifiers_(std::move(modifiers))
{
}

std::vector<pcl::PolygonMesh> CompoundMeshModifier::modify(const pcl::PolygonMesh& original_mesh) const
{
  // Create a vector of meshes as the input and output to each modifier
  // Seed it with the single input mesh for the first modifier
  std::vector<pcl::PolygonMesh> meshes{ original_mesh };

  for (std::size_t i = 0; i < modifiers_.size(); ++i)
  {
    const MeshModifier::ConstPtr& modifier = modifiers_[i];

    std::vector<pcl::PolygonMesh> tmp_meshes;
    for (std::size_t j = 0; j < meshes.size(); ++j)
    {
      try
      {
        const pcl::PolygonMesh& mesh = meshes[j];
        auto tmp = modifier->modify(mesh);
        tmp_meshes.insert(tmp_meshes.end(), tmp.begin(), tmp.end());
      }
      catch (const std::exception& ex)
      {
        std::stringstream ss;
        ss << "Error invoking mesh modifier at index " << i << " on submesh at index " << j << ".";
        std::throw_with_nested(std::runtime_error(ss.str()));
      }
    }

    // Overwrite the meshes for the next modifier
    meshes = tmp_meshes;
  }

  return meshes;
}

}  // namespace noether
