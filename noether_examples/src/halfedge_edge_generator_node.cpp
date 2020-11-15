/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file halfedge_finder_node.cpp
 * @date Dec 5, 2019
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
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <noether_conversions/noether_conversions.h>
#include <tool_path_planner/utilities.h>
#include <visualization_msgs/MarkerArray.h>
#include <noether_msgs/GenerateToolPathsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <console_bridge/console.h>

using RGBA = std::tuple<double, double, double, double>;

static const double GOAL_WAIT_PERIOD = 300.0;  // sec
static const std::string GENERATE_EDGE_PATHS_ACTION = "generate_tool_paths";
static const std::string DEFAULT_FRAME_ID = "world";
static const std::string BOUNDARY_LINES_MARKERS_TOPIC = "boundary_lines";
static const std::string BOUNDARY_POSES_MARKERS_TOPIC = "boundary_poses";
static const std::string EDGE_PATH_NS = "edge_";
static const std::string INPUT_MESH_NS = "input_mesh";
static const RGBA RAW_MESH_RGBA = std::make_tuple(0.6, 0.6, 1.0, 1.0);
static const std::size_t MAX_MARKERS_ON_DISPLAY = 500;

std::size_t countPathPoints(const noether_msgs::ToolPaths& tool_paths)
{
  std::size_t point_count = 0;

  for (const auto& path : tool_paths.paths)
    for (const auto& segment : path.segments)
      point_count += segment.poses.size();

  return point_count;
}

class HalfedgeExample
{
public:
  HalfedgeExample(ros::NodeHandle nh) : nh_(nh), ac_(GENERATE_EDGE_PATHS_ACTION)
  {
    boundary_lines_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(BOUNDARY_LINES_MARKERS_TOPIC, 1);
    boundary_poses_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(BOUNDARY_POSES_MARKERS_TOPIC, 1);
  }

  ~HalfedgeExample() {}

  bool run()
  {
    using namespace noether_conversions;

    // loading parameters
    ros::NodeHandle ph("~");
    std::string mesh_file;
    if (!ph.getParam("mesh_file", mesh_file))
    {
      ROS_ERROR("Failed to load one or more parameters");
      return false;
    }

    markers_publish_timer_ = nh_.createTimer(ros::Duration(0.5), [&](const ros::TimerEvent& e) {
      if (!line_markers_.markers.empty())
      {
        boundary_lines_markers_pub_.publish(line_markers_);
      }

      if (!poses_markers_.markers.empty())
      {
        boundary_poses_markers_pub_.publish(poses_markers_);
      }
    });

    shape_msgs::Mesh mesh_msg;
    if (!noether_conversions::loadPLYFile(mesh_file, mesh_msg))
    {
      ROS_ERROR("Failed to read file %s", mesh_file.c_str());
      return false;
    }
    line_markers_.markers.push_back(createMeshMarker(mesh_file, INPUT_MESH_NS, DEFAULT_FRAME_ID, RAW_MESH_RGBA));

    // waiting for server
    if (!ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_ERROR("Action server %s was not found", GENERATE_EDGE_PATHS_ACTION.c_str());
      return false;
    }

    // sending request
    noether_msgs::GenerateToolPathsGoal goal;
    noether_msgs::ToolPathConfig config;
    config.type = noether_msgs::ToolPathConfig::HALFEDGE_EDGE_GENERATOR;
    tool_path_planner::toHalfedgeConfigMsg(config.halfedge_generator,
                                           tool_path_planner::HalfedgeEdgeGenerator::Config());

    goal.path_configs.push_back(config);
    goal.surface_meshes.push_back(mesh_msg);
    goal.proceed_on_failure = true;
    ac_.sendGoal(goal);
    ros::Time start_time = ros::Time::now();
    if (!ac_.waitForResult(ros::Duration(GOAL_WAIT_PERIOD)))
    {
      ROS_ERROR("Failed to generate edges from mesh");
      return false;
    }

    noether_msgs::GenerateToolPathsResultConstPtr res = ac_.getResult();
    if (!res->success)
    {
      ROS_ERROR("Failed to generate edges from mesh");
      return false;
    }

    const std::vector<noether_msgs::ToolPaths>& boundary_poses = res->tool_paths;

    ROS_INFO("Found %lu edges", boundary_poses.size());

    for (std::size_t i = 0; i < boundary_poses.size(); i++)
    {
      ROS_INFO("Edge %lu contains %lu points", i, countPathPoints(boundary_poses[i]));

      std::string ns = EDGE_PATH_NS + std::to_string(i);
      visualization_msgs::MarkerArray edge_path_axis_markers =
          convertToAxisMarkers(boundary_poses[i], DEFAULT_FRAME_ID, ns);

      visualization_msgs::MarkerArray edge_path_line_markers =
          convertToDottedLineMarker(boundary_poses[i], DEFAULT_FRAME_ID, ns);

      if (poses_markers_.markers.size() > MAX_MARKERS_ON_DISPLAY)
      {
        // prevents buffer overruns
        poses_markers_.markers.clear();
      }

      if (line_markers_.markers.size() > MAX_MARKERS_ON_DISPLAY)  // prevents buffer overruns
      {
        // prevents buffer overruns
        line_markers_.markers.clear();
      }

      poses_markers_.markers.insert(
          poses_markers_.markers.end(), edge_path_axis_markers.markers.begin(), edge_path_axis_markers.markers.end());
      line_markers_.markers.insert(
          line_markers_.markers.end(), edge_path_line_markers.markers.begin(), edge_path_line_markers.markers.end());
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
  actionlib::SimpleActionClient<noether_msgs::GenerateToolPathsAction> ac_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "halfedge_example");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);
  HalfedgeExample halfedge_example(nh);
  if (!halfedge_example.run())
  {
    return -1;
  }
  ros::waitForShutdown();
  return 0;
}
