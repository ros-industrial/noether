/*
 * halfedge_finder_node.cpp
 *
 *  Created on: Dec 5, 2019
 *      Author: jrgnicho
 */

#include <ros/ros.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <noether_conversions/noether_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <tool_path_planner/half_edge_boundary_finder.h>
#include <console_bridge/console.h>

using RGBA = std::tuple<double,double,double,double>;

static const std::string DEFAULT_FRAME_ID = "world";
static const std::string BOUNDARY_LINES_MARKERS_TOPIC ="boundary_lines";
static const std::string BOUNDARY_POSES_MARKERS_TOPIC ="boundary_poses";
static const std::string EDGE_PATH_NS = "edge_";
static const std::string EDGE_PATH_LINE_NS = "edge_path_lines";
static const std::string INPUT_MESH_NS = "input_mesh";
static const RGBA RAW_MESH_RGBA = std::make_tuple(0.6, 0.6, 1.0, 1.0);

class HalfEdgeFinder
{
public:

  HalfEdgeFinder(ros::NodeHandle nh):
    nh_(nh)
  {
    boundary_lines_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(BOUNDARY_LINES_MARKERS_TOPIC,1);
    boundary_poses_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(BOUNDARY_POSES_MARKERS_TOPIC,1);
  }

  ~HalfEdgeFinder()
  {

  }

  bool run()
  {
    using namespace noether_conversions;

    // loading parameters
    ros::NodeHandle ph("~");
    std::string mesh_file;
    int min_num_points = 50;
    double min_point_dist = 0.02;
    if(!ph.getParam("mesh_file",mesh_file))
    {
      ROS_ERROR("Failed to load one or more parameters");
      return false;
    }

    bool loaded_params = ph.param<int>("min_num_points",min_num_points,min_num_points) &&
        ph.param<double>("min_point_dist",min_point_dist,min_point_dist);
    if(!loaded_params)
    {
      ROS_WARN("One or more non-critical parameters were not loaded, using defaults");
    }

    markers_publish_timer_ = nh_.createTimer(ros::Duration(0.5),[&](const ros::TimerEvent& e){
      if(!line_markers_.markers.empty())
      {
        boundary_lines_markers_pub_.publish(line_markers_);
      }

      if(!poses_markers_.markers.empty())
      {
        boundary_poses_markers_pub_.publish(poses_markers_);
      }
    });

    shape_msgs::Mesh mesh_msg;
    if(!noether_conversions::loadPLYFile(mesh_file,mesh_msg))
    {
      ROS_ERROR("Failed to read file %s",mesh_file.c_str());
      return false;
    }
    line_markers_.markers.push_back(createMeshMarker(mesh_file,INPUT_MESH_NS,DEFAULT_FRAME_ID,RAW_MESH_RGBA));

    tool_path_planner::HalfEdgeBoundaryFinder edge_finder;
    decltype(edge_finder)::Config config = {.min_num_points = min_num_points,
                                            .min_point_dist = min_point_dist};
    edge_finder.setInput(mesh_msg);
    ROS_INFO("Computing edges");
    boost::optional<std::vector<geometry_msgs::PoseArray>> res = edge_finder.generate(config);
    const std::vector<geometry_msgs::PoseArray>& boundary_poses = res.get();

    ROS_INFO("Found %lu edges",boundary_poses.size());

    for(std::size_t i = 0; i < boundary_poses.size(); i++)
    {
      ROS_INFO("Edge %lu contains %lu points",i, boundary_poses[i].poses.size());

      std::string ns = EDGE_PATH_NS + std::to_string(i);
      visualization_msgs::MarkerArray edge_path_axis_markers = convertToAxisMarkers({boundary_poses[i]},
                                                                               DEFAULT_FRAME_ID,
                                                                               ns);

      visualization_msgs::MarkerArray edge_path_line_markers = convertToDottedLineMarker({boundary_poses[i]},
                                                                               DEFAULT_FRAME_ID,
                                                                               ns);

      if(poses_markers_.markers.size() > 500)
      {
        poses_markers_.markers.clear();
      }

      if(line_markers_.markers.size() > 500)
      {
        line_markers_.markers.clear();
      }

      poses_markers_.markers.insert( poses_markers_.markers.end(),
                               edge_path_axis_markers.markers.begin(), edge_path_axis_markers.markers.end());
      line_markers_.markers.insert( line_markers_.markers.end(),
                               edge_path_line_markers.markers.begin(), edge_path_line_markers.markers.end());
    }
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::Timer markers_publish_timer_;
  ros::Publisher boundary_lines_markers_pub_;
  ros::Publisher boundary_poses_markers_pub_;
  visualization_msgs::MarkerArray line_markers_;
  visualization_msgs::MarkerArray poses_markers_;



};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"halfedge_finder");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);
  HalfEdgeFinder edge_finder(nh);
  if(!edge_finder.run())
  {
    return -1;
  }
  ros::waitForShutdown();
  return 0;
}





