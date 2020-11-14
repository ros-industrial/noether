/*
 * window_sinc_smoothing.cpp
 *
 *  Created on: Dec 6, 2019
 *      Author: jrgnicho
 */

#include <XmlRpcException.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkWindowedSincPolyDataFilter.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <console_bridge/console.h>
#include "noether_filtering/utils.h"
#include "noether_filtering/mesh/windowed_sinc_smoothing.h"

namespace noether_filtering
{
namespace mesh
{
WindowedSincSmoothing::WindowedSincSmoothing() {}

WindowedSincSmoothing::~WindowedSincSmoothing() {}

bool WindowedSincSmoothing::configure(XmlRpc::XmlRpcValue config)
{
  std::vector<std::string> fields = {
    "num_iter",
    "enable_boundary_smoothing",
    "enable_feature_edge_smoothing",
    "enable_non_manifold_smoothing",
    "enable_normalize_coordinates",
    "feature_angle",
    "edge_angle",
    "pass_band",
  };

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
    config_.num_iter = static_cast<int>(config[fields[idx++]]);
    config_.enable_boundary_smoothing = static_cast<bool>(config[fields[idx++]]);
    config_.enable_feature_edge_smoothing = static_cast<bool>(config[fields[idx++]]);
    config_.enable_non_manifold_smoothing = static_cast<bool>(config[fields[idx++]]);
    config_.enable_normalize_coordinates = static_cast<bool>(config[fields[idx++]]);
    config_.feature_angle = static_cast<double>(config[fields[idx++]]);
    config_.edge_angle = static_cast<double>(config[fields[idx++]]);
    config_.pass_band = static_cast<double>(config[fields[idx++]]);
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    CONSOLE_BRIDGE_logError(e.getMessage().c_str());
    return false;
  }
  return true;
}

bool WindowedSincSmoothing::filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out)
{
  using namespace pcl;

  vtkSmartPointer<vtkPolyData> mesh_data = vtkSmartPointer<vtkPolyData>::New();
  VTKUtils::mesh2vtk(mesh_in, mesh_data);
  std::size_t num_start_points = mesh_data->GetNumberOfPoints();

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

  mesh_data = smoother->GetOutput();
  VTKUtils::vtk2mesh(mesh_data, mesh_out);
  return true;
}

std::string WindowedSincSmoothing::getName() const { return utils::getClassName<decltype(*this)>(); }

} /* namespace mesh */
} /* namespace noether_filtering */
