/*
 * clean_data.cpp
 *
 *  Created on: Dec 6, 2019
 *      Author: jrgnicho
 */

#include <noether_tpp/mesh_modifiers/clean_data_modifier.h>

#include <vtkCleanPolyData.h>
#include <pcl/io/vtk_lib_io.h>
#include <yaml-cpp/yaml.h>

namespace noether
{
std::vector<pcl::PolygonMesh> CleanData::modify(const pcl::PolygonMesh& mesh_in) const
{
  vtkSmartPointer<vtkPolyData> mesh_data = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(mesh_in, mesh_data);

  mesh_data->BuildCells();
  mesh_data->BuildLinks();

  vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
  cleanPolyData->SetInputData(mesh_data);
  cleanPolyData->Update();
  mesh_data = cleanPolyData->GetOutput();

  pcl::PolygonMesh mesh_out;
  pcl::io::vtk2mesh(mesh_data, mesh_out);
  return { mesh_out };
}

}  // namespace noether

namespace YAML
{
Node convert<noether::CleanData>::encode(const T& val) { return {}; }

bool convert<noether::CleanData>::decode(const Node& node, T& val) { return true; }

}  // namespace YAML
