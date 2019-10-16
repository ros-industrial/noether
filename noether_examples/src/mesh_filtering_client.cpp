/*
 * mesh_filtering_client.cpp
 *
 *  Created on: Oct 16, 2019
 *      Author: jrgnicho
 */

#include <ros/ros.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <noether_conversions/noether_conversions.h>
#include <noether_msgs/ApplyMeshFiltersAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/MarkerArray.h>

static const double GOAL_WAIT_PERIOD = 60.0; //sec
static const std::string APPLY_MESH_FILTERS_ACTION = "apply_mesh_filters";
static const std::string DEFAULT_FRAME_ID = "world";
static const std::string MESH_MARKER_TOPIC ="mesh_filtering";
static const std::string RAW_MESH_NS = "raw_mesh";
static const std::string FILTERED_MESH_NS = "filtered_mesh";

visualization_msgs::Marker createMeshMarker(const std::string& mesh_file, const std::string& ns)
{
  visualization_msgs::Marker m;
  m.action = m.ADD;
  m.id = 0;
  m.ns = ns;
  std::tie(m.color.r, m.color.g, m.color.b, m.color.r) = std::make_tuple(0.6, 0.6, 1.0, 1.0);
  m.header.frame_id = DEFAULT_FRAME_ID;
  m.lifetime = ros::Duration(0);
  std::tie(m.scale.x , m.scale.y ,m.scale.z) = std::make_tuple(1.0,1.0,1.0);
  m.mesh_resource = "file://" + mesh_file;
  m.type = m.MESH_RESOURCE;
  return std::move(m);
}

class MeshFilteringClient
{
public:
  MeshFilteringClient(ros::NodeHandle nh):
    nh_(nh),
    ac_(APPLY_MESH_FILTERS_ACTION)
  {
    mesh_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(MESH_MARKER_TOPIC,1);
  }

  ~MeshFilteringClient()
  {

  }

  bool run()
  {
    namespace fs = boost::filesystem;

    pcl::PolygonMesh mesh;
    // loading parameters
    ros::NodeHandle ph("~");
    std::string mesh_file;
    std::string filter_group;
    std::string results_dir;
    if(!ph.getParam("mesh_file",mesh_file) || !ph.getParam("filter_group",filter_group) ||
        !ph.getParam("results_dir", results_dir))
    {
      ROS_ERROR("Failed to load one or more parameters");
      return false;
    }

    ros::Timer mesh_publish_timer = nh_.createTimer(ros::Duration(0.5),[&](const ros::TimerEvent& e){
      for(visualization_msgs::Marker& m: mesh_markers.markers)
      {
        mesh_marker_pub_.publish(m);
      }
    });

    shape_msgs::Mesh mesh_msg;
    if(!noether_conversions::loadPLYFile(mesh_file,mesh_msg))
    {
      ROS_ERROR("Failed to read file %s",mesh_file.c_str());
      return false;
    }
    mesh_markers.markers.push_back(createMeshMarker(mesh_file,RAW_MESH_NS));


    // waiting for server
    if(!ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_ERROR("Action server %s was not found", APPLY_MESH_FILTERS_ACTION.c_str());
      return false;
    }

    // sending request
    noether_msgs::ApplyMeshFiltersGoal goal;
    goal.filter_group = filter_group;
    goal.surface_meshes.push_back(mesh_msg);
    ac_.sendGoal(goal);
    if(!ac_.waitForResult(ros::Duration(GOAL_WAIT_PERIOD)))
    {
      ROS_ERROR("Failed to filter mesh");
      return false;
    }

    // writing out mesh
    static const std::string FILTERED_PREFIX = "filtered_mesh_";
    noether_msgs::ApplyMeshFiltersResultConstPtr res = ac_.getResult();
    int count = 0;
    for(const shape_msgs::Mesh& m : res->filtered_meshes)
    {
      fs::path filtered_mesh_file = fs::path(results_dir) / fs::path(boost::str(boost::format("%s%i.ply") % FILTERED_PREFIX % ++count));
      noether_conversions::savePLYFile(filtered_mesh_file.string(),m);
      mesh_markers.markers.push_back(createMeshMarker(filtered_mesh_file.string(),FILTERED_MESH_NS));
    }

    return true;
  }

private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<noether_msgs::ApplyMeshFiltersAction> ac_;
  ros::Publisher mesh_marker_pub_;
  visualization_msgs::MarkerArray mesh_markers;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv,"mesh_filtering_client");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  MeshFilteringClient client(nh);
  if(!client.run())
  {
    return -1;
  }
  return 0;
}



