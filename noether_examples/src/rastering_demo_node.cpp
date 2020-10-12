/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file rastering_demo_node.cpp
 * @date Dec 27, 2019
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

#include <ros/ros.h>
#include <XmlRpcException.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <noether_conversions/noether_conversions.h>
#include <tool_path_planner/plane_slicer_raster_generator.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <console_bridge/console.h>

using RGBA = std::tuple<double,double,double,double>;

static const std::string DEFAULT_FRAME_ID = "world";
static const std::string RASTER_LINES_MARKERS_TOPIC ="raster_lines";
static const std::string RASTER_POSES_MARKERS_TOPIC ="raster_poses";
static const std::string RASTER_PATH_NS = "raster_";
static const std::string INPUT_MESH_NS = "input_mesh";
static const RGBA RAW_MESH_RGBA = std::make_tuple(0.6, 0.6, 1.0, 1.0);

class Rasterer
{
public:

  Rasterer(ros::NodeHandle nh):
    nh_(nh)
  {
    raster_lines_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(RASTER_LINES_MARKERS_TOPIC,1);
    raster_poses_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(RASTER_POSES_MARKERS_TOPIC,1);
  }

  ~Rasterer()
  {

  }

  bool loadConfig(tool_path_planner::PlaneSlicerRasterGenerator::Config& config)
  {
    ros::NodeHandle ph("~");
    XmlRpc::XmlRpcValue cfg;
    if(!ph.getParam("plane_slicer_rastering", cfg))
    {
      return false;
    }

    /*
     * load configuration into the following struct type
      struct Config
      {
        double raster_spacing = 0.04;
        double point_spacing = 0.01;
        double raster_rot_offset = 0.0;
        double min_hole_size = 0.01;
        double min_segment_size = 0.01;
        double search_radius = 0.01;
      };
    */

    std::vector<std::string> field_names = {"raster_spacing",
      "point_spacing",
      "raster_rot_offset",
      "min_hole_size",
      "min_segment_size",
      "search_radius"};
    if(!std::all_of(field_names.begin(), field_names.end(), [&cfg](const std::string& f){
      return cfg.hasMember(f);
    }))
    {
      ROS_ERROR("Failed to find one or more members");
    }

    std::size_t idx = 0;
    try
    {
      config.raster_spacing = static_cast<double>(cfg[field_names[idx++]]);
      config.point_spacing = static_cast<double>(cfg[field_names[idx++]]);
      config.raster_rot_offset = static_cast<double>(cfg[field_names[idx++]]);
      config.min_hole_size = static_cast<double>(cfg[field_names[idx++]]);
      config.min_segment_size = static_cast<double>(cfg[field_names[idx++]]);
      config.search_radius = static_cast<double>(cfg[field_names[idx++]]);
    }
    catch(XmlRpc::XmlRpcException& e)
    {
      ROS_ERROR_STREAM(e.getMessage());
    }
    return true;
  }

  bool run()
  {
    using namespace noether_conversions;

    // loading parameters
    ros::NodeHandle ph("~");
    std::string mesh_file;
    if(!ph.getParam("mesh_file",mesh_file))
    {
      ROS_ERROR("Failed to load one or more parameters");
      return false;
    }

    ROS_INFO("Got mesh file %s", mesh_file.c_str());

    // get configuration
    tool_path_planner::PlaneSlicerRasterGenerator::Config config;
    if(!loadConfig(config))
    {
      ROS_WARN("Failed to load configuration, using default parameters");
    }

    markers_publish_timer_ = nh_.createTimer(ros::Duration(0.5),[&](const ros::TimerEvent& e){
      if(!line_markers_.markers.empty())
      {
        raster_lines_markers_pub_.publish(line_markers_);
      }

      if(!poses_markers_.markers.empty())
      {
        raster_poses_markers_pub_.publish(poses_markers_);
      }
    });

    shape_msgs::Mesh mesh_msg;
    if(!noether_conversions::loadPLYFile(mesh_file,mesh_msg))
    {
      ROS_ERROR("Failed to read file %s",mesh_file.c_str());
      return false;
    }
    line_markers_.markers.push_back(createMeshMarker(mesh_file,INPUT_MESH_NS,DEFAULT_FRAME_ID,RAW_MESH_RGBA));



    // rastering
    tool_path_planner::PlaneSlicerRasterGenerator raster_gen;
    boost::optional< std::vector<noether_msgs::ToolRasterPath> > temp = raster_gen.generate(mesh_msg,config);
    if(!temp)
    {
      ROS_ERROR("Failed to generate rasters");
      return false;
    }
    std::vector<noether_msgs::ToolRasterPath> raster_paths = *temp;

    ROS_INFO("Found %lu rasters",raster_paths.size());

    for(std::size_t i = 0; i < raster_paths.size(); i++)
    {
      ROS_INFO("Raster %lu contains %lu segments",i, raster_paths[i].paths.size());
      for(std::size_t j = 0; j < raster_paths[i].paths.size(); j++)
      {
        std::cout<<"\tSegment "<< j << " contains "<< raster_paths[i].paths[j].poses.size() << " points"<<std::endl;


        std::string ns = RASTER_PATH_NS + std::to_string(i) + std::string("_s[") + std::to_string(j) + std::string("]") ;
        visualization_msgs::MarkerArray edge_path_axis_markers = convertToAxisMarkers({raster_paths[i].paths[j]},
                                                                                 DEFAULT_FRAME_ID,
                                                                                 ns);

        visualization_msgs::MarkerArray edge_path_line_markers = convertToArrowMarkers({raster_paths[i].paths[j]},
                                                                                 DEFAULT_FRAME_ID,
                                                                                 ns);

        poses_markers_.markers.insert( poses_markers_.markers.end(),
                                 edge_path_axis_markers.markers.begin(), edge_path_axis_markers.markers.end());
        line_markers_.markers.insert( line_markers_.markers.end(),
                                 edge_path_line_markers.markers.begin(), edge_path_line_markers.markers.end());
      }
    }
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::Timer markers_publish_timer_;
  ros::Publisher raster_lines_markers_pub_;
  ros::Publisher raster_poses_markers_pub_;
  visualization_msgs::MarkerArray line_markers_;
  visualization_msgs::MarkerArray poses_markers_;

};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"halfedge_finder");
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  Rasterer edge_finder(nh);
  if(!edge_finder.run())
  {
    return -1;
  }
  ros::waitForShutdown();
  return 0;
}






