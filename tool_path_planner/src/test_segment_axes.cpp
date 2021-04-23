#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <math.h>

#include <tf2_eigen/tf2_eigen.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <thread>
#include <chrono>

#include <tool_path_planner/utilities.h>

tool_path_planner::ToolPathSegment toSegment(std::vector<Eigen::Vector3d> points)
{
  tool_path_planner::ToolPathSegment tool_path_segment;
  for (auto p : points)
  {
    Eigen::Isometry3d point;
    point.translation().x() = p.x();
    point.translation().y() = p.y();
    point.translation().z() = p.z();
    tool_path_segment.push_back(point);
  }
  return tool_path_segment;
}

std::vector<Eigen::Vector3d> applySinWaveToZ(
    std::vector<Eigen::Vector3d> points_in,
    double wave_length=0.4,
    double amplitude=0.1,
    Eigen::Vector3d direction = Eigen::Vector3d(1.0, 1.0, 0))
{
  std::vector<Eigen::Vector3d> points_out;
  for (auto p : points_in)
  {
    p.z() += amplitude * sin(2*M_PI*p.dot(direction) / wave_length);
    points_out.push_back(p);
  }
  return points_out;
}

std::vector<Eigen::Vector3d> applySinWaveByInc(
    std::vector<Eigen::Vector3d> points_in,
    double inc=M_PI/12,
    double phase=0,
    Eigen::Vector3d amplitude=Eigen::Vector3d(0.2, 0, 0))
{
  std::vector<Eigen::Vector3d> points_out;
  double t = phase;
  for (auto p : points_in)
  {
    p.x() += amplitude.x() * sin(t);
    p.y() += amplitude.y() * sin(t);
    p.z() += amplitude.z() * sin(t);
    t += inc;
    points_out.push_back(p);
  }
  return points_out;
}

std::vector<Eigen::Vector3d> projectZToPlane(
    std::vector<Eigen::Vector3d> points_in,
    Eigen::Vector3d plane_coeff)
{
  std::vector<Eigen::Vector3d> points_out;
  for (auto p : points_in)
  {
    double x = plane_coeff.x();
    double y = plane_coeff.y();
    p.z() = -1*x*p.x() + -1*y*p.y();
    points_out.push_back(p);
  }
  return points_out;
}

std::vector<Eigen::Vector3d> circle(
    double radius = 1.0,
    double angle_inc = DEG2RAD(5))
{
  std::vector<Eigen::Vector3d> points;

  for (int i = 0; i < int(std::round(2 * M_PI / angle_inc)); ++i)
  {
    Eigen::Vector3d p;
    double angle = i * angle_inc;
    if (abs(angle - DEG2RAD(0.0)) < 0.001)
    {
      p.x() = radius;
      p.y() = 0;
    }
    else if (abs(angle - DEG2RAD(90.0)) < 0.001)
    {
      p.x() = 0;
      p.y() = radius;
    }
    else if (abs(angle - DEG2RAD(180.0)) < 0.001)
    {
      p.x() = -radius;
      p.y() = 0;
    }
    else if (abs(angle - DEG2RAD(270.0)) < 0.001)
    {
      p.x() = 0;
      p.y() = -radius;
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
      p.x() = x_sign * radius * cos(theta);
      p.y() = y_sign * radius * sin(theta);
    }
    points.push_back(p);
  }
  return points;
}

std::vector<Eigen::Vector3d> polygon(
    std::vector<Eigen::Vector3d> points_in,
    double inc = 0.05)
{
  std::vector<Eigen::Vector3d> points_out;
  for (std::size_t point_index = 0; point_index < points_in.size(); ++point_index)
  {
    double x1 = points_in[point_index].x();
    double y1 = points_in[point_index].y();
    double z1 = points_in[point_index].z();
    double x2 = points_in[(point_index+1)%points_in.size()].x();
    double y2 = points_in[(point_index+1)%points_in.size()].y();
    double z2 = points_in[(point_index+1)%points_in.size()].z();
    double dist = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2) + std::pow(z1 - z2, 2));
    int n = int(std::round(dist / inc));
    double x_inc = (x2 - x1)/n;
    double y_inc = (y2 - y1)/n;
    double z_inc = (z2 - z1)/n;
    for (int i = 0; i < n; ++i)
    {
      Eigen::Vector3d point(
        x1 + (i * x_inc),
        y1 + (i * y_inc),
        z1 + (i * z_inc)
      );
      points_out.push_back(point);
    }
  }
  return points_out;
}

std::vector<Eigen::Vector3d> rectangle(
    double length=1.0,
    double width=2.0,
    double inc=0.05)
{
  return polygon(
    {
      Eigen::Vector3d(-length/2, -width/2, 0),
      Eigen::Vector3d(-length/2,  width/2, 0),
      Eigen::Vector3d( length/2,  width/2, 0),
      Eigen::Vector3d( length/2, -width/2, 0)
    },
    inc
  );
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_segment_axes");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("segments", 10);
  ros::Publisher eigen_pub_1 = nh.advertise<visualization_msgs::Marker>("eigen_1", 10);
  ros::Publisher eigen_pub_2 = nh.advertise<visualization_msgs::Marker>("eigen_2", 10);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // create tool path
  tool_path_planner::ToolPaths tool_paths;
  tool_path_planner::ToolPath tool_path;

  std::vector<Eigen::Vector3d> points;
//  points = polygon(
//    {
//      Eigen::Vector3d(-1.0 , -0.8 , 0),
//      Eigen::Vector3d(-0.2 ,  0   , 0),
//      Eigen::Vector3d(-1.0 ,  0.5 , 0),
//      Eigen::Vector3d(-0.5 ,  0.5 , 0),
//      Eigen::Vector3d(-0.6 ,  1.0 , 0),
//      Eigen::Vector3d( 0   ,  0.5 , 0),
//      Eigen::Vector3d( 1   ,  0.5 , 0),
//      Eigen::Vector3d( 0.5 ,  0   , 0),
//      Eigen::Vector3d( 0   , -1   , 0)
//    },
//    0.001
//  );
//  points = rectangle(1, 2, 0.01);
  points = circle(1.0, 0.01);

//  points = projectZToPlane(points, Eigen::Vector3d(1.0, 1.0, 0));
//  points = applySinWaveByInc(points, M_PI/120, 0, Eigen::Vector3d(0.1, 0, 0));
//  points = applySinWaveByInc(points, M_PI/120, M_PI/2, Eigen::Vector3d(0, 0.0, 0.1));
//  points = applySinWaveToZ(points, 0.1, 0.1, Eigen::Vector3d(0.1, 0, 0));

  tool_path.push_back(toSegment(points));

  tool_paths.push_back(tool_path);

  tool_paths = tool_path_planner::segmentByAxes(tool_paths, Eigen::Vector3f(1.0, 0, 0), Eigen::Vector3f(0.0, 1.0, 0.0));

  std::size_t n = tool_paths[0].size();
  std::vector<std::vector<float>> colors = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0},
    {1.0, 1.0, 0.0}
  };

  int marker_id = 2;
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
      marker.scale.x = 0.04;
      marker.scale.y = 0.04;
      marker.scale.z = 0.04;
      marker_array.markers.push_back(marker);
//      pub.publish(marker_array);
//      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      ++marker_id;
    }
  }

  pub.publish(marker_array);

  ROS_ERROR_STREAM("DONE.");

  ros::spin();

  return 0;
}
