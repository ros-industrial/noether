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

#include <tool_path_planner/path_generator.h>

#include <boost/optional.hpp>
#include <jsoncpp/json/json.h>
#include <pcl/PolygonMesh.h>

namespace tool_path_planner
{
class [[deprecated("Replaced by PlaneSliceRasterPlanner in noether_tpp")]] PlaneSlicerRasterGenerator
  : public PathGenerator
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

    Json::Value toJson() const;
    bool fromJson(const Json::Value& jv);
    bool fromJson(const std::string& jv_string);
    std::string str();
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
  pcl::PolygonMesh mesh_data_;
  Config config_;
};

} /* namespace tool_path_planner */

#endif /* INCLUDE_PLANE_SLICER_RASTER_GENERATOR_H_ */
