/*
 * window_sinc_smoothing.cpp
 *
 *  Created on: Dec 6, 2019
 *      Author: jrgnicho
 */

#include <noether_tpp/mesh_modifiers/windowed_sinc_smoothing_modifier.h>
#include <noether_tpp/serialization.h>

#include <vtkWindowedSincPolyDataFilter.h>
#include <pcl/io/vtk_lib_io.h>

namespace noether
{
std::vector<pcl::PolygonMesh> WindowedSincSmoothing::modify(const pcl::PolygonMesh& mesh_in) const
{
  vtkSmartPointer<vtkPolyData> mesh_data = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(mesh_in, mesh_data);

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
  pcl::io::vtk2mesh(smoother->GetOutput(), mesh_out);
  return { mesh_out };
}

}  // namespace noether

namespace YAML
{
Node convert<noether::WindowedSincSmoothing>::encode(const T& val)
{
  Node node;
  node["num_iter"] = val.config_.num_iter;
  node["enable_boundary_smoothing"] = val.config_.enable_boundary_smoothing;
  node["enable_feature_edge_smoothing"] = val.config_.enable_feature_edge_smoothing;
  node["enable_non_manifold_smoothing"] = val.config_.enable_non_manifold_smoothing;
  node["enable_normalize_coordinates"] = val.config_.enable_normalize_coordinates;
  node["feature_angle"] = val.config_.feature_angle;
  node["edge_angle"] = val.config_.edge_angle;
  node["pass_band"] = val.config_.pass_band;
  return node;
}

bool convert<noether::WindowedSincSmoothing>::decode(const Node& node, T& val)
{
  val.config_.num_iter = static_cast<std::size_t>(getMember<int>(node, "num_iter"));
  val.config_.enable_boundary_smoothing = getMember<bool>(node, "enable_boundary_smoothing");
  val.config_.enable_feature_edge_smoothing = getMember<bool>(node, "enable_feature_edge_smoothing");
  val.config_.enable_non_manifold_smoothing = getMember<bool>(node, "enable_non_manifold_smoothing");
  val.config_.enable_normalize_coordinates = getMember<bool>(node, "enable_normalize_coordinates");
  val.config_.feature_angle = getMember<double>(node, "feature_angle");
  val.config_.edge_angle = getMember<double>(node, "edge_angle");
  val.config_.pass_band = getMember<double>(node, "pass_band");
  return true;
}

}  // namespace YAML
