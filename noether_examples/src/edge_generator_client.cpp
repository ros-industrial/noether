/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file edge_generator_client.cpp
 * @date Nov 14, 2019
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
#include <noether_msgs/GenerateEdgePathsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/MarkerArray.h>


using RGBA = std::tuple<double,double,double,double>;

static const double GOAL_WAIT_PERIOD = 300.0; //sec
static const std::string GENERATE_EDGE_PATHS_ACTION = "generate_edge_paths";
static const std::string DEFAULT_FRAME_ID = "world";
static const std::string EDGE_GEN_MARKERS_TOPIC ="edge_generation";
static const std::string EDGE_PATH_AXIS_NS = "edge_paths_axis";
static const std::string EDGE_PATH_LINE_NS = "edge_path_lines";
static const std::string INPUT_MESH_NS = "input_mesh";
static const RGBA RAW_MESH_RGBA = std::make_tuple(0.6, 0.6, 1.0, 1.0);

class EdgeGeneratorClient
{
public:

  EdgeGeneratorClient(ros::NodeHandle nh):
    nh_(nh),
    ac_(GENERATE_EDGE_PATHS_ACTION)
  {
    mesh_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(EDGE_GEN_MARKERS_TOPIC,1);
  }

  ~EdgeGeneratorClient()
  {

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

    markers_publish_timer_ = nh_.createTimer(ros::Duration(0.5),[&](const ros::TimerEvent& e){
      mesh_markers_pub_.publish(markers_);
    });

    shape_msgs::Mesh mesh_msg;
    if(!noether_conversions::loadPLYFile(mesh_file,mesh_msg))
    {
      ROS_ERROR("Failed to read file %s",mesh_file.c_str());
      return false;
    }
    markers_.markers.push_back(createMeshMarker(mesh_file,INPUT_MESH_NS,DEFAULT_FRAME_ID,RAW_MESH_RGBA));

    // waiting for server
    if(!ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_ERROR("Action server %s was not found", GENERATE_EDGE_PATHS_ACTION.c_str());
      return false;
    }

    // sending request
    noether_msgs::GenerateEdgePathsGoal goal;
    goal.proceed_on_failure = true;
    goal.surface_meshes.push_back(mesh_msg);
    ac_.sendGoal(goal);
    ros::Time start_time = ros::Time::now();
    if(!ac_.waitForResult(ros::Duration(GOAL_WAIT_PERIOD)))
    {
      ROS_ERROR("Failed to generate edges from mesh");
      return false;
    }

    noether_msgs::GenerateEdgePathsResultConstPtr res = ac_.getResult();
    if(!res->success)
    {
      ROS_ERROR("Failed to generate edges from mesh");
      return false;
    }

    for(std::size_t i = 0; i < res->edge_paths.size(); i++)
    {
      std::string ns = EDGE_PATH_AXIS_NS + std::to_string(i);
      visualization_msgs::MarkerArray edge_path_axis_markers = convertToAxisMarkers(res->edge_paths[i],
                                                                               DEFAULT_FRAME_ID,
                                                                               ns);
      ns = EDGE_PATH_LINE_NS + std::to_string(i);
      visualization_msgs::MarkerArray edge_path_line_markers = convertToDottedLineMarker(res->edge_paths[i],
                                                                               DEFAULT_FRAME_ID,
                                                                               ns);

      markers_.markers.insert( markers_.markers.end(),
                               edge_path_axis_markers.markers.begin(), edge_path_axis_markers.markers.end());
      markers_.markers.insert( markers_.markers.end(),
                               edge_path_line_markers.markers.begin(), edge_path_line_markers.markers.end());
    }

    return true;
  }

private:

  ros::NodeHandle nh_;
  ros::Timer markers_publish_timer_;
  actionlib::SimpleActionClient<noether_msgs::GenerateEdgePathsAction> ac_;
  ros::Publisher mesh_markers_pub_;
  visualization_msgs::MarkerArray markers_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv,"edge_generator_client");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  EdgeGeneratorClient client(nh);
  if(!client.run())
  {
    return -1;
  }
  ros::waitForShutdown();
  return 0;
}


