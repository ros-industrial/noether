/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file fill_holes.pp
 * @date Dec 16, 2019
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkFillHolesFilter.h>
#include <vtkPolyDataNormals.h>
#include <XmlRpcException.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <console_bridge/console.h>
#include "noether_filtering/mesh/fill_holes.h"
#include "noether_filtering/utils.h"

namespace noether_filtering
{
namespace mesh
{
FillHoles::FillHoles() {}

FillHoles::~FillHoles() {}

bool FillHoles::configure(XmlRpc::XmlRpcValue config)
{
  std::vector<std::string> fields = { "hole_size" };

  if (!std::all_of(fields.begin(), fields.end(), [&config, this](const std::string& f) {
        if (!config.hasMember(f))
        {
          CONSOLE_BRIDGE_logError("The %s config field %s was not found", getName().c_str(), f.c_str());
          return false;
        }
        return true;
      }))
  {
    return false;
  }

  try
  {
    std::size_t idx = 0;
    hole_size_ = static_cast<double>(config[fields[idx++]]);
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    CONSOLE_BRIDGE_logError(e.getMessage().c_str());
    return false;
  }
  return true;
}

bool FillHoles::filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out)
{
  using namespace pcl;

  vtkSmartPointer<vtkPolyData> mesh_data = vtkSmartPointer<vtkPolyData>::New();
  VTKUtils::mesh2vtk(mesh_in, mesh_data);
  std::size_t start_num_polys = mesh_data->GetNumberOfPolys();

  vtkSmartPointer<vtkFillHolesFilter> fill_holes_filter = vtkSmartPointer<vtkFillHolesFilter>::New();
  fill_holes_filter->SetInputData(mesh_data);
  fill_holes_filter->SetHoleSize(hole_size_);
  fill_holes_filter->Update();

  vtkSmartPointer<vtkPolyDataNormals> normals_rectifier = vtkSmartPointer<vtkPolyDataNormals>::New();
  normals_rectifier->SetInputData(fill_holes_filter->GetOutput());
  normals_rectifier->ConsistencyOn();
  normals_rectifier->SplittingOff();
  normals_rectifier->Update();

  mesh_data = normals_rectifier->GetOutput();
  std::size_t end_num_polys = mesh_data->GetNumberOfPolys();
  CONSOLE_BRIDGE_logInform("Filled %lu polygons", start_num_polys - end_num_polys);
  VTKUtils::vtk2mesh(mesh_data, mesh_out);
  return true;
}

std::string FillHoles::getName() const { return utils::getClassName<decltype(*this)>(); }

} /* namespace mesh */
} /* namespace noether_filtering */
