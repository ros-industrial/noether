#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <math.h>

#include <tf2_eigen/tf2_eigen.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <thread>
#include <chrono>

#include <tool_path_planner/utilities.h>


int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_segment_axes");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("segments", 10);
  std::this_thread::sleep_for(std::chrono::seconds(10));

  // create tool path
  tool_path_planner::ToolPaths tool_paths;
  tool_path_planner::ToolPath tool_path;
  tool_path_planner::ToolPathSegment tool_path_segment;

  double radius = 1.0;
  double angle_inc = DEG2RAD(15);

  for (int i = 0; i < int(std::round(2 * M_PI / angle_inc)); ++i)
  {
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    double angle = i * angle_inc;
    if (abs(angle - DEG2RAD(0.0)) < 0.001)
    {
      p.translation().x() = radius;
      p.translation().y() = 0;
    }
    else if (abs(angle - DEG2RAD(90.0)) < 0.001)
    {
      p.translation().x() = 0;
      p.translation().y() = radius;
    }
    else if (abs(angle - DEG2RAD(180.0)) < 0.001)
    {
      p.translation().x() = -radius;
      p.translation().y() = 0;
    }
    else if (abs(angle - DEG2RAD(270.0)) < 0.001)
    {
      p.translation().x() = 0;
      p.translation().y() = -radius;
    }
    else
    {
      double theta = angle;
      int y_sign = 1;
      int x_sign = 1;
      if (DEG2RAD(90.0) < angle < DEG2RAD(180.0))
      {
        theta = DEG2RAD(180.0) - angle;
        x_sign = -1;
      }
      else if (DEG2RAD(180.0) < angle < DEG2RAD(270.0))
      {
        theta = angle - DEG2RAD(180.0);
        y_sign = -1;
        x_sign = -1;
      }
      else if (DEG2RAD(270.0) < angle < DEG2RAD(360.0))
      {
        theta = DEG2RAD(360.0) - angle;
        y_sign = -1;
      }
      p.translation().x() = x_sign * radius * cos(theta);
      p.translation().y() = y_sign * radius * sin(theta);
    }
    tool_path_segment.push_back(p);
  }

  tool_path.push_back(tool_path_segment);
  tool_paths.push_back(tool_path);

  tool_paths = tool_path_planner::segmentByAxes(tool_paths);

  std::size_t n = tool_paths[0].size();
  std::vector<std::vector<float>> colors = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0},
    {1.0, 1.0, 0.0}
  };

  int marker_id = 0;
  visualization_msgs::MarkerArray marker_array;
  for (std::size_t i = 0; i < n; ++i)
  {
    for (Eigen::Isometry3d pose : tool_paths[0][i])
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      marker.ns = "";
      marker.id = marker_id;
      marker.color.a = 1.0;
      marker.color.r = colors[i][0];
      marker.color.g = colors[i][1];
      marker.color.b = colors[i][2];
      marker.pose = tf2::toMsg(pose);
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = 0.02;
      marker.scale.y = 0.02;
      marker.scale.z = 0.02;
      marker_array.markers.push_back(marker);
      pub.publish(marker_array);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      ++marker_id;
    }
  }

//  pub.publish(marker_array);

  ROS_ERROR_STREAM("DONE.");

  ros::spin();

  return 0;
}
