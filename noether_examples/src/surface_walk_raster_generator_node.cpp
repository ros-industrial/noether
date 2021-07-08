#include <ros/ros.h>
#include <XmlRpcException.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <noether_conversions/noether_conversions.h>
#include <tool_path_planner/surface_walk_raster_generator.h>
#include <tool_path_planner/utilities.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <console_bridge/console.h>
#include <noether_msgs/GenerateToolPathsAction.h>

using RGBA = std::tuple<double, double, double, double>;

static const double GOAL_WAIT_PERIOD = 300.0;  // sec
static const std::string DEFAULT_FRAME_ID = "world";
static const std::string GENERATE_TOOL_PATHS_ACTION = "generate_tool_paths";
static const std::string RASTER_LINES_MARKERS_TOPIC = "raster_lines";
static const std::string RASTER_POSES_MARKERS_TOPIC = "raster_poses";
static const std::string RASTER_PATH_NS = "raster_";
static const std::string INPUT_MESH_NS = "input_mesh";
static const RGBA RAW_MESH_RGBA = std::make_tuple(0.6, 0.6, 1.0, 1.0);

std::size_t countPathPoints(const noether_msgs::ToolPaths& tool_paths)
{
    std::size_t point_count = 0;

    for (const auto& path : tool_paths.paths)
        for (const auto& segment : path.segments)
            point_count += segment.poses.size();

    return point_count;
}

class SurfaceWalkExample
{
public:
    SurfaceWalkExample(ros::NodeHandle nh) : nh_(nh), ac_(GENERATE_TOOL_PATHS_ACTION)
    {
        raster_lines_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(RASTER_LINES_MARKERS_TOPIC, 1);
        raster_poses_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(RASTER_POSES_MARKERS_TOPIC, 1);
    }

    ~SurfaceWalkExample() {}

    bool loadConfig(tool_path_planner::SurfaceWalkRasterGenerator::Config& config)
    {
        ros::NodeHandle ph("~");
        XmlRpc::XmlRpcValue cfg;
        if (!ph.getParam("surface_walk_rastering", cfg))
        {
            return false;
        }

        std::vector<std::string> field_names = { "raster_spacing", "point_spacing", "raster_rot_offset",
                                                 "min_hole_size",  "min_segment_size", "intersection_plane_height",
                                                 "tool_offset", "generate_extra_rasters", "cut_direction_x",
                                                 "cut_direction_y", "cut_direction_z", "debug"};
        if (!std::all_of(field_names.begin(), field_names.end(), [&cfg](const std::string& f) { return cfg.hasMember(f); }))
        {
            ROS_ERROR("Failed to find one or more members");
        }

        std::size_t idx = 0;
        try
        {
            config.raster_spacing = static_cast<double>(cfg[field_names[idx++]]);
            config.point_spacing = static_cast<double>(cfg[field_names[idx++]]);
            config.raster_rot_offset = static_cast<double>(cfg[field_names[idx++]]);
            config.min_hole_size = static_cast<double>(cfg[field_names[idx++]]);
            config.min_segment_size = static_cast<double>(cfg[field_names[idx++]]);
            config.intersection_plane_height = static_cast<double>(cfg[field_names[idx++]]);
            config.tool_offset = static_cast<double>(cfg[field_names[idx++]]);
            config.generate_extra_rasters = static_cast<bool>(cfg[field_names[idx++]]);
            for(int i=0; i < 3; i++)
            {
                config.cut_direction[i] = static_cast<double>(cfg[field_names[idx++]]);
            }
            config.debug = static_cast<bool>(cfg[field_names[idx++]]);
        }
        catch (XmlRpc::XmlRpcException& e)
        {
            ROS_ERROR_STREAM(e.getMessage());
        }
        return true;
    }

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

        ROS_INFO("Got mesh file %s", mesh_file.c_str());

        // get configuration
        tool_path_planner::SurfaceWalkRasterGenerator::Config config;
        if (!loadConfig(config))
        {
            ROS_WARN("Failed to load configuration, using default parameters");
        }

        markers_publish_timer_ = nh_.createTimer(ros::Duration(0.5), [&](const ros::TimerEvent& e) {
            if (!line_markers_.markers.empty())
            {
                raster_lines_markers_pub_.publish(line_markers_);
            }

            if (!poses_markers_.markers.empty())
            {
                raster_poses_markers_pub_.publish(poses_markers_);
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
            ROS_ERROR("Action server %s was not found", GENERATE_TOOL_PATHS_ACTION.c_str());
            return false;
        }

        // sending request
        noether_msgs::GenerateToolPathsGoal goal;
        noether_msgs::ToolPathConfig config_msg;
        config_msg.type = noether_msgs::ToolPathConfig::SURFACE_WALK_RASTER_GENERATOR;
        tool_path_planner::toSurfaceWalkConfigMsg(config_msg.surface_walk_generator, config);

        goal.path_configs.push_back(config_msg);
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

        const std::vector<noether_msgs::ToolPaths>& raster_paths = res->tool_paths;

        ROS_INFO("Found %lu rasters", raster_paths.size());

        for (std::size_t i = 0; i < raster_paths.size(); i++)
        {
            ROS_INFO("Raster %lu contains %lu points", i, countPathPoints(raster_paths[i]));

            std::string ns = RASTER_PATH_NS + std::to_string(i);
            visualization_msgs::MarkerArray edge_path_axis_markers =
                    convertToAxisMarkers(raster_paths[i], DEFAULT_FRAME_ID, ns);

            visualization_msgs::MarkerArray edge_path_line_markers =
                    convertToArrowMarkers(raster_paths[i], DEFAULT_FRAME_ID, ns);

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
    ros::Publisher raster_lines_markers_pub_;
    ros::Publisher raster_poses_markers_pub_;
    visualization_msgs::MarkerArray line_markers_;
    visualization_msgs::MarkerArray poses_markers_;
    actionlib::SimpleActionClient<noether_msgs::GenerateToolPathsAction> ac_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "surface_walk_example_node");
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    SurfaceWalkExample surface_walk_example(nh);
    if (!surface_walk_example.run())
    {
        return -1;
    }
    ros::waitForShutdown();
    return 0;
}
