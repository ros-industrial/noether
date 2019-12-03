/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file edge_generator_server.cpp
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
#include <console_bridge/console.h>
#include <XmlRpcException.h>
#include <actionlib/server/simple_action_server.h>
#include <noether_msgs/GenerateEdgePathsAction.h>
#include <noether_conversions/noether_conversions.h>
#include <tool_path_planner/edge_path_generator.h>

static const double FEEDBACK_PUBLISH_PERIOD = 1.0; // seconds
static const std::string GENERATE_EDGE_PATHS_ACTION = "generate_edge_paths";
static const std::string EDGE_PATH_CONFIG_PARAM = "edge_path_config";

class EdgeGeneratorServer
{
public:
  EdgeGeneratorServer():
    server_(nh_, GENERATE_EDGE_PATHS_ACTION, boost::bind(&EdgeGeneratorServer::executeAction, this, _1),false)
  {

  }

  ~EdgeGeneratorServer()
  {

  }

  bool run()
  {
    if(!loadConfig())
    {
      return false;
    }
    server_.start();
    ROS_INFO("Edge Generator Server is ready ...");
    return true;
  }

protected:

  bool loadConfig()
  {
    using namespace XmlRpc;

    ros::NodeHandle ph("~");
    XmlRpcValue config;
    if(!ph.getParam(EDGE_PATH_CONFIG_PARAM,config))
    {
      ROS_ERROR("Did not find the '%s' parameter", EDGE_PATH_CONFIG_PARAM.c_str());
      return false;
    }

    try
    {
      edge_gen_config_.octree_res = static_cast<double>(config["octree_res"]);
      edge_gen_config_.search_radius = static_cast<double>(config["search_radius"]);
      edge_gen_config_.num_threads = static_cast<int>(config["num_threads"]);

      edge_gen_config_.neighbor_tol = static_cast<double>(config["neighbor_tol"]);

      edge_gen_config_.edge_cluster_min = static_cast<int>(config["edge_cluster_min"]);
      edge_gen_config_.voxel_size = static_cast<double>(config["voxel_size"]);
      edge_gen_config_.kdtree_epsilon = static_cast<double>(config["kdtree_epsilon"]);

      edge_gen_config_.max_intersecting_voxels = static_cast<int>(config["max_intersecting_voxels"]);
      edge_gen_config_.min_projection_dist = static_cast<double>(config["min_projection_dist"]);

    }
    catch(XmlRpcException& e)
    {
      ROS_ERROR("Fail to parse edge generation config file, error msg: %s", e.getMessage().c_str());
      return false;
    }
    return true;
  }

  void executeAction(const noether_msgs::GenerateEdgePathsGoalConstPtr& goal)
  {
    using namespace noether_msgs;
    noether_msgs::GenerateEdgePathsFeedback feedback;
    noether_msgs::GenerateEdgePathsResult res;

    if(!loadConfig())
    {
      ROS_WARN("Could not load configuration, using previous one");
    }

    ros::Timer feedback_timer = nh_.createTimer(ros::Duration(),[&](const ros::TimerEvent& evnt)
    {
      server_.publishFeedback(feedback);
    });

    for(std::size_t i = 0; i < goal->surface_meshes.size(); i++)
    {
      auto& mesh = goal->surface_meshes[i];
      feedback.current_mesh_index = i;
      boost::optional< std::vector<geometry_msgs::PoseArray> > edge_path_poses = edge_path_gen_.generate(mesh,edge_gen_config_);
      if(!edge_path_poses)
      {
        res.validities.push_back(false);
        res.edge_paths.push_back(ToolRasterPath());

        if(!goal->proceed_on_failure)
        {
          res.success = false;
          break;
        }
      }
      else
      {
        ToolRasterPath edge_path;
        edge_path.paths = edge_path_poses.get();
        res.edge_paths.push_back(std::move(edge_path));
        res.validities.push_back(true);
      }
    }

    if(goal->proceed_on_failure)
    {
      res.success = std::any_of(res.validities.begin(), res.validities.end(),[](const bool& b){
        return b;
      });
    }

    if(res.success)
    {
      server_.setSucceeded(res);
    }
    else
    {
      server_.setAborted(res);
    }
  }

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<noether_msgs::GenerateEdgePathsAction> server_;
  tool_path_planner::EdgePathConfig edge_gen_config_;
  tool_path_planner::EdgePathGenerator edge_path_gen_;

};

int main(int argc,char** argv)
{
  ros::init(argc,argv,"edge_generator_server");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);
  EdgeGeneratorServer server;
  if(!server.run())
  {
    return -1;
  }

  ros::waitForShutdown();
  return 0;
}

