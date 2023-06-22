/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file plane_slicer_raster_generator.cpp
 * @date Dec 26, 2019
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

#include <tool_path_planner/plane_slicer_raster_generator.h>
#include <tool_path_planner/utilities.h>

#include <boost/make_shared.hpp>
#include <console_bridge/console.h>
#include <noether_conversions/noether_conversions.h>
#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/centroid_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators/principal_axis_direction_generator.h>
#include <numeric>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>

namespace tool_path_planner
{
Json::Value PlaneSlicerRasterGenerator::Config::toJson() const
{
  Json::Value jv(Json::ValueType::objectValue);
  jv["raster_spacing"] = raster_spacing;
  jv["point_spacing"] = point_spacing;
  jv["raster_rot_offset"] = raster_rot_offset;
  jv["min_segment_size"] = min_segment_size;
  jv["search_radius"] = search_radius;
  jv["min_hole_size"] = min_hole_size;

  return jv;
}

bool PlaneSlicerRasterGenerator::Config::fromJson(const Json::Value& jv)
{
  if (jv.isNull())
  {
    CONSOLE_BRIDGE_logError("Json value is null");
    return false;
  }
  if (jv.type() != Json::ValueType::objectValue)
  {
    CONSOLE_BRIDGE_logError("Json type %i is invalid, only '%i' is allowed",
                            static_cast<int>(jv.type()),
                            static_cast<int>(Json::ValueType::objectValue));
    return false;
  }
  auto validate = [](const Json::Value& jv, const std::string& name_, const Json::ValueType& type_) -> bool {
    return jv.isMember(name_) && jv[name_].type() == type_;
  };
  raster_spacing = validate(jv, "raster_spacing", Json::ValueType::realValue) ? jv["raster_spacing"].asDouble() :
                                                                                DEFAULT_RASTER_SPACING;
  point_spacing = validate(jv, "point_spacing", Json::ValueType::realValue) ? jv["point_spacing"].asDouble() :
                                                                              DEFAULT_POINT_SPACING;
  raster_rot_offset = validate(jv, "raster_rot_offset", Json::ValueType::realValue) ?
                          jv["raster_rot_offset"].asDouble() :
                          DEFAULT_RASTER_ROT_OFFSET;
  min_segment_size = validate(jv, "min_segment_size", Json::ValueType::realValue) ? jv["min_segment_size"].asDouble() :
                                                                                    DEFAULT_MIN_SEGMENT_SIZE;
  search_radius = validate(jv, "search_radius", Json::ValueType::realValue) ? jv["search_radius"].asDouble() :
                                                                              DEFAULT_SEARCH_RADIUS;
  min_hole_size = validate(jv, "min_hole_size", Json::ValueType::realValue) ? jv["min_hole_size"].asDouble() :
                                                                              DEFAULT_MIN_HOLE_SIZE;
  return true;
}

bool PlaneSlicerRasterGenerator::Config::fromJson(const std::string& jv_string)
{
  Json::Value jv;
  Json::Reader r;
  if (!r.parse(jv_string, jv))
    return false;

  return fromJson(jv);
}

std::string PlaneSlicerRasterGenerator::Config::str()
{
  std::stringstream ss;
  ss << "raster_spacing: " << raster_spacing << std::endl;
  ss << "point_spacing: " << point_spacing << std::endl;
  ss << "raster_rot_offset: " << raster_rot_offset << std::endl;
  ss << "min_segment_size: " << min_segment_size << std::endl;
  ss << "search_radius: " << search_radius << std::endl;
  return ss.str();
}

void PlaneSlicerRasterGenerator::setConfiguration(const PlaneSlicerRasterGenerator::Config& config)
{
  config_ = config;
}

void PlaneSlicerRasterGenerator::setInput(pcl::PolygonMesh::ConstPtr mesh) { mesh_data_ = *mesh; }

void PlaneSlicerRasterGenerator::setInput(vtkSmartPointer<vtkPolyData> mesh)
{
  if (!mesh)
    throw std::runtime_error("Input mesh is null");

  // Compute normals if none exist
  if (!mesh->GetPointData()->GetNormals() || !mesh->GetCellData()->GetNormals())
  {
    vtkSmartPointer<vtkPolyDataNormals> normal_generator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normal_generator->SetInputData(mesh);
    normal_generator->ComputePointNormalsOn();
    normal_generator->SetComputeCellNormals(!mesh->GetCellData()->GetNormals());
    normal_generator->SetFeatureAngle(M_PI_2);
    normal_generator->SetSplitting(true);
    normal_generator->SetConsistency(true);
    normal_generator->SetAutoOrientNormals(false);
    normal_generator->SetFlipNormals(false);
    normal_generator->SetNonManifoldTraversal(false);
    normal_generator->Update();

    if (!mesh->GetPointData()->GetNormals())
    {
      mesh->GetPointData()->SetNormals(normal_generator->GetOutput()->GetPointData()->GetNormals());
    }

    if (!mesh->GetCellData()->GetNormals())
    {
      mesh->GetCellData()->SetNormals(normal_generator->GetOutput()->GetCellData()->GetNormals());
    }
  }

  pcl::VTKUtils::vtk2mesh(mesh, mesh_data_);
}

void PlaneSlicerRasterGenerator::setInput(const shape_msgs::Mesh& mesh)
{
  pcl::PolygonMesh::Ptr pcl_mesh = boost::make_shared<pcl::PolygonMesh>();
  noether_conversions::convertToPCLMesh(mesh, *pcl_mesh);
  setInput(pcl_mesh);
}

vtkSmartPointer<vtkPolyData> PlaneSlicerRasterGenerator::getInput()
{
  auto vtk_mesh = vtkSmartPointer<vtkPolyData>::New();
  pcl::VTKUtils::mesh2vtk(mesh_data_, vtk_mesh);
  return vtk_mesh;
}

boost::optional<ToolPaths> PlaneSlicerRasterGenerator::generate()
{
  try
  {
    noether::PlaneSlicerRasterPlanner planner(
        std::make_unique<noether::PrincipalAxisDirectionGenerator>(config_.raster_rot_offset),
        std::make_unique<noether::CentroidOriginGenerator>());
    planner.setLineSpacing(config_.raster_spacing);
    planner.setPointSpacing(config_.point_spacing);
    planner.setMinHoleSize(config_.min_hole_size);
    planner.setSearchRadius(config_.search_radius);
    planner.setMinSegmentSize(config_.min_segment_size);

    return planner.plan(mesh_data_);
  }
  catch (const std::exception& ex)
  {
    return {};
  }
}

std::string PlaneSlicerRasterGenerator::getName() const { return getClassName<decltype(*this)>(); }

} /* namespace tool_path_planner */
