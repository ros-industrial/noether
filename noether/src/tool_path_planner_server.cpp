#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <noether_msgs/GenerateToolPathsAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/format.hpp>
#include <mutex>
#include <thread>
#include <tool_path_planner/utilities.h>
#include <tool_path_planner/path_generator.h>
#include <tool_path_planner/surface_walk_raster_generator.h>
#include <tool_path_planner/plane_slicer_raster_generator.h>
#include <tool_path_planner/eigen_value_edge_generator.h>
#include <tool_path_planner/halfedge_edge_generator.h>

static const double FEEDBACK_PUBLISH_PERIOD = 1.0;  // seconds
static const std::string GENERATE_TOOL_PATHS_ACTION = "generate_tool_paths";

namespace tpp_path_gen
{
using GenPathActionServer = actionlib::SimpleActionServer<noether_msgs::GenerateToolPathsAction>;

class TppServer
{
public:
  TppServer(ros::NodeHandle nh, std::string action_name) : as_(nh, action_name, false) {}

  ~TppServer() {}

  void start()
  {
    as_.registerGoalCallback(boost::bind(&TppServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&TppServer::preemptCB, this));
    as_.start();
  }

protected:
  void runPathGeneration(const GenPathActionServer::GoalConstPtr goal)
  {
    ROS_INFO("Starting path generation");
    {
      std::lock_guard<std::mutex> lock(goal_process_mutex_);
      // validating data
      if (goal->surface_meshes.empty())
      {
        std::string err_msg = "No surfaces were received";
        ROS_ERROR_STREAM(err_msg);
        noether_msgs::GenerateToolPathsResult result;
        as_.setAborted(result, err_msg);
        return;
      }

      // verifying configs
      if (goal->path_configs.empty())
      {
        std::string err_msg = "Path configuration array is empty, using default values";
        ROS_ERROR_STREAM(err_msg);
        noether_msgs::GenerateToolPathsResult result;
        as_.setAborted(result, err_msg);
        return;
      }

      std::vector<noether_msgs::ToolPathConfig> tpp_configs;
      tpp_configs.assign(goal->path_configs.begin(), goal->path_configs.end());

      if (tpp_configs.size() != goal->surface_meshes.size())
      {
        std::string err_msg = "Surface meshes and path configs array have unequal sizes";
        ROS_ERROR_STREAM(err_msg);
        noether_msgs::GenerateToolPathsResult result;
        as_.setAborted(result, err_msg);
        return;
      }
    }
    noether_msgs::GenerateToolPathsFeedback feedback;
    ros::Timer feedback_timer =
        nh_.createTimer(ros::Duration(), [&](const ros::TimerEvent& evnt) { as_.publishFeedback(feedback); });

    const std::size_t num_meshes = goal->surface_meshes.size();
    std::vector<bool> validities;
    using ToolPath = std::vector<geometry_msgs::PoseArray>;
    noether_msgs::GenerateToolPathsResult result;
    result.tool_path_validities.resize(num_meshes, false);
    result.tool_paths.resize(num_meshes);
    for (std::size_t i = 0; i < goal->surface_meshes.size(); i++)
    {
      {
        std::lock_guard<std::mutex> lock(goal_process_mutex_);
        feedback.current_mesh_index = i;
        if (as_.isPreemptRequested())
        {
          ROS_WARN("Canceling Tool Path Generation Request");
          break;
        }
      }

      tool_path_planner::PathGenerator::Ptr generator = nullptr;
      const noether_msgs::ToolPathConfig& config = goal->path_configs[i];
      const shape_msgs::Mesh& mesh = goal->surface_meshes[i];
      boost::optional<tool_path_planner::ToolPaths> tool_paths = boost::none;
      ROS_INFO("Planning path");
      if (config.type == noether_msgs::ToolPathConfig::SURFACE_WALK_RASTER_GENERATOR)
      {
        auto path_gen = std::make_shared<tool_path_planner::SurfaceWalkRasterGenerator>();
        tool_path_planner::SurfaceWalkRasterGenerator::Config path_config;
        tool_path_planner::toSurfaceWalkConfig(path_config, config.surface_walk_generator);
        path_gen->setConfiguration(path_config);
        generator = path_gen;
      }
      else if (config.type == noether_msgs::ToolPathConfig::PLANE_SLICER_RASTER_GENERATOR)
      {
        auto path_gen = std::make_shared<tool_path_planner::PlaneSlicerRasterGenerator>();
        tool_path_planner::PlaneSlicerRasterGenerator::Config path_config;
        tool_path_planner::toPlaneSlicerConfig(path_config, config.plane_slicer_generator);
        path_gen->setConfiguration(path_config);
        generator = path_gen;
      }
      else if (config.type == noether_msgs::ToolPathConfig::EIGEN_VALUE_EDGE_GENERATOR)
      {
        auto path_gen = std::make_shared<tool_path_planner::EigenValueEdgeGenerator>();
        tool_path_planner::EigenValueEdgeGenerator::Config path_config;
        tool_path_planner::toEigenValueConfig(path_config, config.eigen_value_generator);
        path_gen->setConfiguration(path_config);
        generator = path_gen;
      }
      else if (config.type == noether_msgs::ToolPathConfig::HALFEDGE_EDGE_GENERATOR)
      {
        auto path_gen = std::make_shared<tool_path_planner::HalfedgeEdgeGenerator>();
        tool_path_planner::HalfedgeEdgeGenerator::Config path_config;
        tool_path_planner::toHalfedgeConfig(path_config, config.halfedge_generator);
        path_gen->setConfiguration(path_config);
        generator = path_gen;
      }
      else
      {
        std::string err_msg = "Unsupported request type!";
        ROS_ERROR_STREAM(err_msg);
        noether_msgs::GenerateToolPathsResult result;
        as_.setAborted(result, err_msg);
        return;
      }

      generator->setInput(mesh);
      tool_paths = generator->generate();

      if (tool_paths.is_initialized())
      {
        noether_msgs::ToolPaths trp;
        for (const auto& tool_path : tool_paths.get())
        {
          noether_msgs::ToolPath tp;
          for (const auto& segment : tool_path)
          {
            geometry_msgs::PoseArray seg;
            for (const auto& p : segment)
            {
              geometry_msgs::Pose pose;
              tf::poseEigenToMsg(p, pose);
              seg.poses.push_back(pose);
            }
            tp.segments.push_back(seg);
          }
          trp.paths.push_back(tp);
        }

	result.tool_paths[i] = trp;
        result.tool_path_validities[i] = true;

        std::string ros_info_msg = boost::str(boost::format("Surface %1% processed successfully") % (i + 1));
        ROS_INFO_STREAM(ros_info_msg);
      }
      else
      {
        std::string err_msg = boost::str(boost::format("Path planning on surface %1% failed") % (i + 1));
        ROS_ERROR_STREAM(err_msg);
        if (!goal->proceed_on_failure)
        {
          break;
        }
      }
    }

    result.success = std::any_of(
        result.tool_path_validities.begin(), result.tool_path_validities.end(), [](const bool& b) { return b; });

    {
      std::lock_guard<std::mutex> lock(goal_process_mutex_);
      if (result.success)
      {
        as_.setSucceeded(result);
      }
      else
      {
        as_.setAborted(result);
      }
    }
  }

  void goalCB()
  {
    GenPathActionServer::GoalConstPtr goal;
    {
      std::lock_guard<std::mutex> lock(goal_process_mutex_);

      if (as_.isActive())
      {
        ROS_ERROR("Currently Processing a TPP request, rejecting");
        return;
      }

      goal = as_.acceptNewGoal();
      if (goal == nullptr)
      {
        std::string err_msg = "Goal ptr received is invalid";
        ROS_ERROR_STREAM(err_msg);
        noether_msgs::GenerateToolPathsResult result;
        as_.setAborted(result, err_msg);
        return;
      }
    }

    runPathGeneration(goal);
  }

  void preemptCB()
  {
    std::lock_guard<std::mutex> lock(goal_process_mutex_);
    as_.setPreempted();
  }

  ros::NodeHandle nh_;
  GenPathActionServer as_;
  std::mutex goal_process_mutex_;
};

}  // namespace tpp_path_gen

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tool_path_planner_node");
  ros::AsyncSpinner spinner(4);
  ros::NodeHandle nh;
  spinner.start();
  tpp_path_gen::TppServer tpp_server(nh, GENERATE_TOOL_PATHS_ACTION);
  tpp_server.start();
  ros::waitForShutdown();
  return 0;
}
