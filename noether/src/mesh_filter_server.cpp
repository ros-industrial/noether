/*
 * mesh_filter_server.cpp
 *
 *  Created on: Oct 15, 2019
 *      Author: Jorge Nicho
 *      Email: jrgnichodevel@gmail.com
 */

#include <ros/ros.h>
#include <boost/format.hpp>
#include <actionlib/server/simple_action_server.h>
#include <noether_msgs/ApplyMeshFiltersAction.h>
#include <noether_filtering/mesh_filter_manager.h>
#include <noether_conversions/noether_conversions.h>

static const std::string APPLY_MESH_FILTERS_ACTION = "apply_mesh_filters";
static const std::string MESH_FILTER_MNGR_PARAM = "mesh_filter_manager";

class MeshFilterServer
{
public:
  MeshFilterServer():
    server_(nh_, APPLY_MESH_FILTERS_ACTION, boost::bind(&MeshFilterServer::executeAction, this, _1),false)
  {

  }

  ~MeshFilterServer()
  {

  }

  bool run()
  {
    ros::NodeHandle ph("~");
    XmlRpc::XmlRpcValue config = ph.param<XmlRpc::XmlRpcValue>(MESH_FILTER_MNGR_PARAM,
                                                               XmlRpc::XmlRpcValue());
    if(!filter_manager_.init(config))
    {
      ROS_ERROR("Mesh Filter Server failed to initialize");
      return false;
    }
    server_.start();
    ROS_INFO("Mesh Filter Server is ready ...");
    return true;
  }

private:

  void executeAction(const noether_msgs::ApplyMeshFiltersGoalConstPtr& goal)
  {
    using namespace noether_msgs;
    ApplyMeshFiltersResult res;
    ApplyMeshFiltersFeedback feedback;

    res.success = false;
    std::shared_ptr< noether_filtering::MeshFilterGroup > mesh_filter_group = filter_manager_.getFilterGroup(goal->filter_group);
    if(mesh_filter_group == nullptr)
    {
      std::string err_msg = boost::str(boost::format("The filter group '%s' was not found") % goal->filter_group);
      server_.setAborted(res,err_msg);
      return;
    }

    // creating timer for publishing feedback
    ros::Timer feedback_pub_timer = nh_.createTimer(ros::Duration(0.5),[&](const ros::TimerEvent& evnt){
      server_.publishFeedback(feedback);
    });

    // applying filters one mesh at a time
    for(std::size_t i = 0; i < goal->surface_meshes.size(); i++)
    {
      feedback.current_mesh_index = i;

      pcl::PolygonMesh mesh_in, mesh_out;
      noether_conversions::convertToPCLMesh(goal->surface_meshes[i],mesh_in);
      std::string err_msg;
      if(!mesh_filter_group->applyFilters(goal->custom_filter_names,mesh_in,mesh_out, err_msg))
      {
        server_.setAborted(res,err_msg);
        return;
      }
      ROS_INFO("Filtered mesh %lu", i);
      shape_msgs::Mesh filtered_mesh_msg;
      noether_conversions::convertToMeshMsg(mesh_out,filtered_mesh_msg);
      res.filtered_meshes.push_back(std::move(filtered_mesh_msg));
    }
    res.success = true;
    server_.setSucceeded(res);
  }

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<noether_msgs::ApplyMeshFiltersAction> server_;
  noether_filtering::MeshFilterManager filter_manager_;
};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"mesh_filter_server");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  MeshFilterServer server;
  if(!server.run())
  {
    return -1;
  }

  ros::waitForShutdown();
  return 0;
}

