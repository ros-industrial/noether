/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file mesh_filter_server.cpp
 * @date Oct 15, 2019
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
#include <actionlib/server/simple_action_server.h>
#include <noether_msgs/ApplyMeshFiltersAction.h>
#include <noether_conversions/noether_conversions.h>
#include <noether_filtering/mesh/mesh_filter_manager.h>

static const std::string APPLY_MESH_FILTERS_ACTION = "apply_mesh_filters";
static const std::string MESH_FILTER_MNGR_PARAM = "mesh_filter_manager";

class MeshFilterServer
{
public:
  MeshFilterServer()
    : server_(nh_, APPLY_MESH_FILTERS_ACTION, boost::bind(&MeshFilterServer::executeAction, this, _1), false)
  {
  }

  ~MeshFilterServer() {}

  bool run()
  {
    if (!loadConfig())
    {
      return false;
    }

    server_.start();
    ROS_INFO("Mesh Filter Server is ready ...");
    return true;
  }

private:
  bool loadConfig()
  {
    ros::NodeHandle ph("~");
    XmlRpc::XmlRpcValue config;
    if (!ph.getParam(MESH_FILTER_MNGR_PARAM, config))
    {
      ROS_ERROR("Mesh Filter Server did not find the '%s' parameter",
                ph.resolveName(MESH_FILTER_MNGR_PARAM, true).c_str());
      return false;
    }

    if (!filter_manager_.init(config))
    {
      ROS_ERROR("Mesh Filter Server failed to initialize the filter manager");
      return false;
    }
    ROS_INFO("Mesh Filter Server loaded parameters");
    return true;
  }

  void executeAction(const noether_msgs::ApplyMeshFiltersGoalConstPtr& goal)
  {
    using namespace noether_msgs;
    ApplyMeshFiltersResult res;
    ApplyMeshFiltersFeedback feedback;

    if (!loadConfig())
    {
      std::string err_msg = boost::str(boost::format("Mesh Filter Server failed to load parameters"));
      ROS_ERROR_STREAM(err_msg);
      server_.setAborted(res, err_msg);
      return;
    }

    res.success = false;
    std::shared_ptr<noether_filtering::mesh::MeshFilterGroup> mesh_filter_group =
        filter_manager_.getFilterGroup(goal->filter_group);
    if (mesh_filter_group == nullptr)
    {
      std::string err_msg = boost::str(boost::format("The filter group '%s' was not found") % goal->filter_group);
      server_.setAborted(res, err_msg);
      return;
    }

    // creating timer for publishing feedback
    ros::Timer feedback_pub_timer =
        nh_.createTimer(ros::Duration(0.5), [&](const ros::TimerEvent& evnt) { server_.publishFeedback(feedback); });

    // applying filters one mesh at a time
    for (std::size_t i = 0; i < goal->surface_meshes.size(); i++)
    {
      feedback.current_mesh_index = i;

      pcl::PolygonMesh mesh_in, mesh_out;
      noether_conversions::convertToPCLMesh(goal->surface_meshes[i], mesh_in);
      std::string err_msg;
      if (!mesh_filter_group->applyFilters(goal->custom_filter_names, mesh_in, mesh_out, err_msg))
      {
        std::string name = goal->label + "[" + std::to_string(i) + "]";
        server_.setAborted(res, err_msg);
        ROS_ERROR("Failed to filter %s mesh", name.c_str());
        return;
      }
      ROS_INFO("Filtered mesh %lu", i);
      shape_msgs::Mesh filtered_mesh_msg;
      noether_conversions::convertToMeshMsg(mesh_out, filtered_mesh_msg);
      res.filtered_meshes.push_back(std::move(filtered_mesh_msg));
    }
    res.success = true;
    server_.setSucceeded(res);
  }

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<noether_msgs::ApplyMeshFiltersAction> server_;
  noether_filtering::mesh::MeshFilterManager filter_manager_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_filter_server");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);
  MeshFilterServer server;
  if (!server.run())
  {
    return -1;
  }

  ros::waitForShutdown();
  return 0;
}
