/*
 * clean_data.cpp
 *
 *  Created on: Dec 6, 2019
 *      Author: jrgnicho
 */

#include <XmlRpcException.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkCleanPolyData.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <console_bridge/console.h>
#include "noether_filtering/mesh/clean_data.h"
#include "noether_filtering/utils.h"

namespace noether_filtering
{
namespace mesh
{
CleanData::CleanData() {}

CleanData::~CleanData() {}

bool CleanData::configure(XmlRpc::XmlRpcValue config)
{
  // no configuration needed
  return true;
}

bool CleanData::filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out)
{
  using namespace pcl;

  vtkSmartPointer<vtkPolyData> mesh_data = vtkSmartPointer<vtkPolyData>::New();
  VTKUtils::mesh2vtk(mesh_in, mesh_data);
  std::size_t num_start_points = mesh_data->GetNumberOfPoints();

  mesh_data->BuildCells();
  mesh_data->BuildLinks();

  vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
  cleanPolyData->SetInputData(mesh_data);
  cleanPolyData->Update();
  mesh_data = cleanPolyData->GetOutput();
  std::size_t num_end_points = mesh_data->GetNumberOfPoints();
  CONSOLE_BRIDGE_logInform("Removed duplicate points, retained %lu points from %lu", num_end_points, num_start_points);
  VTKUtils::vtk2mesh(mesh_data, mesh_out);
  return true;
}

std::string CleanData::getName() const { return utils::getClassName<decltype(*this)>(); }

} /* namespace mesh */
} /* namespace noether_filtering */
