#include <noether_tpp/mesh_modifiers/compound_modifier.h>

namespace noether
{
CompoundMeshModifier::CompoundMeshModifier(std::vector<MeshModifier::ConstPtr> modifiers)
  : modifiers_(std::move(modifiers))
{
}

std::vector<pcl::PolygonMesh> CompoundMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Create a vector of meshes as the input and output to each modifier
  // Seed it with the single input mesh for the first modifier
  std::vector<pcl::PolygonMesh> meshes{ mesh };

  for (const MeshModifier::ConstPtr& modifier : modifiers_)
  {
    std::vector<pcl::PolygonMesh> tmp_meshes;
    for (const pcl::PolygonMesh& mesh : meshes)
    {
      auto tmp = modifier->modify(mesh);
      tmp_meshes.insert(tmp_meshes.end(), tmp.begin(), tmp.end());
    }

    // Overwrite the meshes for the next modifier
    meshes = tmp_meshes;
  }

  return meshes;
}

}  // namespace noether
