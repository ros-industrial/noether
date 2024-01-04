/*
 * window_sinc_smoothing.cpp
 *
 *  Created on: Dec 6, 2019
 *      Author: jrgnicho
 */

#include <noether_tpp/mesh_modifiers/windowed_sinc_smoothing_modifier.h>

#include <vtkWindowedSincPolyDataFilter.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

namespace noether
{
std::vector<pcl::PolygonMesh> WindowedSincSmoothing::modify(const pcl::PolygonMesh& mesh_in) const
{
  vtkSmartPointer<vtkPolyData> mesh_data = vtkSmartPointer<vtkPolyData>::New();
  pcl::VTKUtils::mesh2vtk(mesh_in, mesh_data);

  mesh_data->BuildCells();
  mesh_data->BuildLinks();

  vtkSmartPointer<vtkWindowedSincPolyDataFilter> smoother = vtkSmartPointer<vtkWindowedSincPolyDataFilter>::New();
  smoother->SetInputData(mesh_data);
  smoother->SetNumberOfIterations(config_.num_iter);
  smoother->SetFeatureAngle(config_.feature_angle);
  smoother->SetEdgeAngle(config_.edge_angle);
  smoother->SetPassBand(config_.pass_band);
  smoother->SetBoundarySmoothing(config_.enable_boundary_smoothing);
  smoother->SetFeatureEdgeSmoothing(config_.enable_feature_edge_smoothing);
  smoother->SetNonManifoldSmoothing(config_.enable_non_manifold_smoothing);
  smoother->SetNormalizeCoordinates(config_.enable_normalize_coordinates);
  smoother->Update();

  pcl::PolygonMesh mesh_out;
  pcl::VTKUtils::vtk2mesh(smoother->GetOutput(), mesh_out);
  return { mesh_out };
}

}  // namespace noether
