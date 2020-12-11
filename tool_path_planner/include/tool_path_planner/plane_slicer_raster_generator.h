/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file plane_slicer_raster_generator.h
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

#ifndef INCLUDE_PLANE_SLICER_RASTER_GENERATOR_H_
#define INCLUDE_PLANE_SLICER_RASTER_GENERATOR_H_

#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <boost/optional.hpp>
#include <pcl/PolygonMesh.h>
#include <shape_msgs/Mesh.h>
#include <vtkCellLocator.h>
#include <vtkKdTreePointLocator.h>
#include <jsoncpp/json/json.h>
#include <ros/console.h>

#include <tool_path_planner/path_generator.h>

namespace tool_path_planner
{
class PlaneSlicerRasterGenerator : public PathGenerator
{
  static constexpr double DEFAULT_RASTER_SPACING = 0.04;
  static constexpr double DEFAULT_POINT_SPACING = 0.01;
  static constexpr double DEFAULT_RASTER_ROT_OFFSET = 0.0;
  static constexpr double DEFAULT_MIN_SEGMENT_SIZE = 0.01;
  static constexpr double DEFAULT_SEARCH_RADIUS = 0.01;
  static constexpr double DEFAULT_MIN_HOLE_SIZE = 1e-2;

public:
  struct Config
  {
    double raster_spacing{ DEFAULT_RASTER_SPACING };
    double point_spacing{ DEFAULT_POINT_SPACING };
    double raster_rot_offset{ DEFAULT_RASTER_ROT_OFFSET };
    double min_segment_size{ DEFAULT_MIN_SEGMENT_SIZE };
    double search_radius{ DEFAULT_SEARCH_RADIUS };
    double min_hole_size{ DEFAULT_MIN_HOLE_SIZE };

    Json::Value toJson() const
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

    bool fromJson(const Json::Value& jv)
    {
      if (jv.isNull())
      {
        ROS_ERROR("Json value is null");
        return false;
      }
      if (jv.type() != Json::ValueType::objectValue)
      {
        ROS_ERROR("Json type %i is invalid, only '%i' is allowed",
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
      min_segment_size = validate(jv, "min_segment_size", Json::ValueType::realValue) ?
                             jv["min_segment_size"].asDouble() :
                             DEFAULT_MIN_SEGMENT_SIZE;
      search_radius = validate(jv, "search_radius", Json::ValueType::realValue) ? jv["search_radius"].asDouble() :
                                                                                  DEFAULT_SEARCH_RADIUS;
      min_hole_size = validate(jv, "min_hole_size", Json::ValueType::realValue) ? jv["min_hole_size"].asDouble() :
                                                                                  DEFAULT_MIN_HOLE_SIZE;
      return true;
    }

    bool fromJson(const std::string& jv_string)
    {
      Json::Value jv;
      Json::Reader r;
      if (!r.parse(jv_string, jv))
        return false;

      return fromJson(jv);
    }

    std::string str()
    {
      std::stringstream ss;
      ss << "raster_spacing: " << raster_spacing << std::endl;
      ss << "point_spacing: " << point_spacing << std::endl;
      ss << "raster_rot_offset: " << raster_rot_offset << std::endl;
      ss << "min_segment_size: " << min_segment_size << std::endl;
      ss << "search_radius: " << search_radius << std::endl;
      return ss.str();
    }
  };

  PlaneSlicerRasterGenerator() = default;

  /**
   * @brief Set the generator configuration
   * @param config The configuration
   * @return True if valid configuration, otherwise false.
   */
  void setConfiguration(const Config& config);

  void setInput(pcl::PolygonMesh::ConstPtr mesh) override;

  void setInput(vtkSmartPointer<vtkPolyData> mesh) override;

  void setInput(const shape_msgs::Mesh& mesh) override;

  vtkSmartPointer<vtkPolyData> getInput() override;

  boost::optional<ToolPaths> generate() override;

  std::string getName() const override;

private:
  bool insertNormals(const double search_radius, vtkSmartPointer<vtkPolyData>& data);

  vtkSmartPointer<vtkPolyData> mesh_data_;
  vtkSmartPointer<vtkKdTreePointLocator> kd_tree_;
  vtkSmartPointer<vtkCellLocator> cell_locator_;
  Config config_;
};

} /* namespace tool_path_planner */

#endif /* INCLUDE_PLANE_SLICER_RASTER_GENERATOR_H_ */
