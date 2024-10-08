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

#include <noether_tpp/mesh_modifiers/fill_holes_modifier.h>

#include <vtkFillHolesFilter.h>
#include <vtkPolyDataNormals.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

namespace noether
{
FillHoles::FillHoles(const double max_hole_size) : max_hole_size_(max_hole_size) {}

std::vector<pcl::PolygonMesh> FillHoles::modify(const pcl::PolygonMesh& mesh_in) const
{
  vtkSmartPointer<vtkPolyData> mesh_data = vtkSmartPointer<vtkPolyData>::New();
  pcl::VTKUtils::mesh2vtk(mesh_in, mesh_data);

  vtkSmartPointer<vtkFillHolesFilter> fill_holes_filter = vtkSmartPointer<vtkFillHolesFilter>::New();
  fill_holes_filter->SetInputData(mesh_data);
  fill_holes_filter->SetHoleSize(max_hole_size_);
  fill_holes_filter->Update();

  vtkSmartPointer<vtkPolyDataNormals> normals_rectifier = vtkSmartPointer<vtkPolyDataNormals>::New();
  normals_rectifier->SetInputData(fill_holes_filter->GetOutput());
  normals_rectifier->ConsistencyOn();
  normals_rectifier->SplittingOff();
  normals_rectifier->Update();

  pcl::PolygonMesh mesh_out;
  pcl::VTKUtils::vtk2mesh(normals_rectifier->GetOutput(), mesh_out);
  return { mesh_out };
}

}  // namespace noether
