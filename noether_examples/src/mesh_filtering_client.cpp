/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file mesh_filtering_client.cpp
 * @date Oct 16, 2019
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
#include <noether_msgs/ApplyMeshFiltersAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/MarkerArray.h>

static const double GOAL_WAIT_PERIOD = 300.0;  // sec
static const std::string APPLY_MESH_FILTERS_ACTION = "apply_mesh_filters";
static const std::string DEFAULT_FRAME_ID = "world";
static const std::string MESH_MARKER_TOPIC = "mesh_filtering";
static const std::string RAW_MESH_NS = "raw_mesh";
static const std::string FILTERED_MESH_NS = "filtered_mesh";
static const std::string FILTERED_PREFIX = "filtered_mesh_";

using RGBA = std::tuple<double, double, double, double>;
namespace mesh_colors
{
static const RGBA RAW_MESH = std::make_tuple(0.6, 0.6, 1.0, 1.0);
static const RGBA FILTERED_MESH = std::make_tuple(0.6, 1.0, 0.4, 0.5);
}  // namespace mesh_colors

class MeshFilteringClient
{
public:
  MeshFilteringClient(ros::NodeHandle nh) : nh_(nh), ac_(APPLY_MESH_FILTERS_ACTION)
  {
    mesh_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(MESH_MARKER_TOPIC, 1);
  }

  ~MeshFilteringClient() {}

  bool run()
  {
    namespace fs = boost::filesystem;

    // loading parameters
    ros::NodeHandle ph("~");
    std::string mesh_file;
    std::string filter_group;
    std::string results_dir;
    if (!ph.getParam("mesh_file", mesh_file) || !ph.getParam("filter_group", filter_group) ||
        !ph.getParam("results_dir", results_dir))
    {
      ROS_ERROR("Failed to load one or more parameters");
      return false;
    }

    mesh_publish_timer_ = nh_.createTimer(ros::Duration(0.5), [&](const ros::TimerEvent& e) {
      for (visualization_msgs::Marker& m : mesh_markers_.markers)
      {
        mesh_marker_pub_.publish(m);
      }
    });

    shape_msgs::Mesh mesh_msg;
    if (!noether_conversions::loadPLYFile(mesh_file, mesh_msg))
    {
      ROS_ERROR("Failed to read file %s", mesh_file.c_str());
      return false;
    }
    mesh_markers_.markers.push_back(
        noether_conversions::createMeshMarker(mesh_file, RAW_MESH_NS, DEFAULT_FRAME_ID, mesh_colors::RAW_MESH));

    // waiting for server
    if (!ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_ERROR("Action server %s was not found", APPLY_MESH_FILTERS_ACTION.c_str());
      return false;
    }

    // sending request
    noether_msgs::ApplyMeshFiltersGoal goal;
    goal.filter_group = filter_group;
    goal.surface_meshes.push_back(mesh_msg);
    ac_.sendGoal(goal);
    ros::Time start_time = ros::Time::now();
    if (!ac_.waitForResult(ros::Duration(GOAL_WAIT_PERIOD)))
    {
      ROS_ERROR("Failed to filter mesh");
      return false;
    }
    ros::Duration time_elapsed = ros::Time::now() - start_time;
    ROS_INFO("Got filtered results, process took %f seconds", time_elapsed.toSec());

    // writing out mesh
    noether_msgs::ApplyMeshFiltersResultConstPtr res = ac_.getResult();
    int count = 0;
    for (const shape_msgs::Mesh& m : res->filtered_meshes)
    {
      fs::path filtered_mesh_file =
          fs::path(results_dir) / fs::path(boost::str(boost::format("%s%i.ply") % FILTERED_PREFIX % ++count));
      ROS_INFO("Saving filtered mesh to location %s", filtered_mesh_file.c_str());
      noether_conversions::savePLYFile(filtered_mesh_file.string(), m);
      mesh_markers_.markers.push_back(noether_conversions::createMeshMarker(
          filtered_mesh_file.string(), FILTERED_MESH_NS, DEFAULT_FRAME_ID, mesh_colors::FILTERED_MESH));
    }

    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::Timer mesh_publish_timer_;
  actionlib::SimpleActionClient<noether_msgs::ApplyMeshFiltersAction> ac_;
  ros::Publisher mesh_marker_pub_;
  visualization_msgs::MarkerArray mesh_markers_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_filtering_client");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  MeshFilteringClient client(nh);
  if (!client.run())
  {
    return -1;
  }
  ros::waitForShutdown();
  return 0;
}
