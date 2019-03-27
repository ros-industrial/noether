#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <noether_msgs/GenerateToolPathsAction.h>
#include <boost/format.hpp>
#include <mutex>
#include <thread>
#include <tool_path_planner/utilities.h>
#include <tool_path_planner/raster_path_generator.h>


static const std::string GENERATE_TOOL_PATHS_ACTION = "generate_tool_paths";

namespace tpp_path_gen
{

using GenPathActionServer = actionlib::SimpleActionServer<noether_msgs::GenerateToolPathsAction>;

class TppServer
{
public:
  TppServer(ros::NodeHandle nh, std::string action_name):
    as_(nh,action_name,false)
  {

  }

  ~TppServer()
  {

  }

  void start()
  {
    as_.registerGoalCallback(boost::bind(&TppServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&TppServer::preemptCB, this));
    as_.start();
  }

protected:

  void runPathGeneration(const GenPathActionServer::GoalConstPtr goal)
  {
    using namespace tool_path_planner;
    ROS_INFO("Starting path generation");
    {
      std::lock_guard<std::mutex> lock(goal_process_mutex_);
      // validating data
      if(goal->surface_meshes.empty())
      {
        std::string err_msg = "No surfaces were received";
        ROS_ERROR_STREAM(err_msg);
        noether_msgs::GenerateToolPathsResult result;
        as_.setAborted(result,err_msg);
        return;
      }


      // verifying configs
      std::vector<noether_msgs::ToolPathConfig> tpp_configs;
      if(goal->path_configs.empty())
      {
        ROS_WARN("Path configuration array is empty, using default values");
        tpp_configs.resize(goal->surface_meshes.size(),toTppMsg(RasterPathGenerator::generateDefaultToolConfig()));
      }
      else
      {
        tpp_configs.assign(goal->path_configs.begin(),goal->path_configs.end());
      }

      if(tpp_configs.size() != goal->surface_meshes.size())
      {
        std::string err_msg = "Surface meshes and path configs array have unequal sizes";
        ROS_ERROR_STREAM(err_msg);
        noether_msgs::GenerateToolPathsResult result;
        as_.setAborted(result,err_msg);
        return;
      }
    }

    const std::size_t num_meshes = goal->surface_meshes.size();
    std::vector<bool> validities;
    using ToolPath = std::vector<geometry_msgs::PoseArray>;
    noether_msgs::GenerateToolPathsResult result;
    result.tool_path_validities.resize(num_meshes,false);
    result.tool_raster_paths.resize(num_meshes);
    for(std::size_t i = 0; i < goal->surface_meshes.size(); i++)
    {
      {
        std::lock_guard<std::mutex> lock(goal_process_mutex_);
        if(as_.isPreemptRequested())
        {
          ROS_WARN("Canceling Tool Path Generation Request");
          break;
        }
      }

      const shape_msgs::Mesh& mesh = goal->surface_meshes[i];
      const noether_msgs::ToolPathConfig& config = goal->path_configs[i];
      tool_path_planner::RasterPathGenerator path_gen;
      ROS_INFO("Planning path");
      boost::optional<ToolPath> tool_path = path_gen.generate(fromTppMsg(config),mesh);

      if(tool_path.is_initialized())
      {
        noether_msgs::ToolRasterPath trp;
        trp.paths = std::move(tool_path.get());
        result.tool_raster_paths[i] = std::move(trp);
        result.tool_path_validities[i] = true;

        std::string ros_info_msg = boost::str(boost::format("Surface %1% processed successfully") % (i+1));
        ROS_INFO_STREAM(ros_info_msg);
      }
      else
      {
        std::string err_msg = boost::str(boost::format("Path planning on surface %1% failed") % (i+1));
        ROS_ERROR_STREAM(err_msg);
        if(!goal->proceed_on_failure)
        {
          break;
        }
      }
    }

    result.success = std::any_of(result.tool_path_validities.begin(),result.tool_path_validities.end(),[](const bool& b){
      return b;
    });

    {
      std::lock_guard<std::mutex> lock(goal_process_mutex_);
      if(result.success)
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

      if(as_.isActive())
      {
        ROS_ERROR("Currently Processing a TPP request, rejecting");
        return;
      }

      goal = as_.acceptNewGoal();
      if(goal == nullptr)
      {
        std::string err_msg = "Goal ptr received is invalid";
        ROS_ERROR_STREAM(err_msg);
        noether_msgs::GenerateToolPathsResult result;
        as_.setAborted(result,err_msg);
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

  GenPathActionServer as_;
  std::mutex goal_process_mutex_;

};

}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"path_generator");
  ros::AsyncSpinner spinner(4);
  ros::NodeHandle nh;
  spinner.start();
  tpp_path_gen::TppServer tpp_server(nh,GENERATE_TOOL_PATHS_ACTION);
  tpp_server.start();
  ros::waitForShutdown();
  return 0;
}
