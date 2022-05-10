/*
 * Copyright (c) 2018, Southwest Research Institute
 * All rights reserved.*
 * utilities.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: Jorge Nicho
 */

#include <limits>
#include <cmath>
#include <numeric>
#include <algorithm> // std::sort

#include <boost/make_shared.hpp>
#include <Eigen/Core>
#include <vtkParametricFunctionSource.h>
#include <vtkOBBTree.h>
#include <vtkIntersectionPolyDataFilter.h>
#include <vtkDelaunay2D.h>
#include <vtkMath.h>
#include <vtkSpline.h>
#include <vtkPolyDataNormals.h>
#include <vtkKdTreePointLocator.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkTriangle.h>
#include <vtkDoubleArray.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtk_viewer/vtk_utils.h>
#include <vtkReverseSense.h>
#include <vtkImplicitDataSet.h>
#include <vtkCutter.h>
#include <vtkCellLocator.h>
#include <vtkGenericCell.h>
#include <vtkTriangleFilter.h>
#include <tool_path_planner/utilities.h>
#include <console_bridge/console.h>
#include <pcl/features/moment_of_inertia_estimation.h>

namespace tool_path_planner
{
typedef std::tuple<double, Eigen::Isometry3d, int> wp_tuple;
typedef std::list<wp_tuple> tlist;
typedef std::list<wp_tuple>::iterator tlist_it;

static const double EPSILON = 1e-3;

void flipPointOrder(ToolPath& path)
{
  // Reverse the order of the segments
  std::reverse(std::begin(path), std::end(path));

  // Reverse the order of the points in each segment
  for (auto& s : path)
    std::reverse(std::begin(s), std::end(s));

  // Next rotate each pose around the z-axis 180 degrees
  for (auto& s : path)
    for (auto& p : s)
      p *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
}

tool_path_planner::ToolPathSegmentData toToolPathSegmentData(const tool_path_planner::ToolPathSegment& tool_path_segment)
{
  using namespace Eigen;

  tool_path_planner::ToolPathSegmentData tool_path_segment_data;
  tool_path_segment_data.line = vtkSmartPointer<vtkPolyData>::New();
  tool_path_segment_data.derivatives = vtkSmartPointer<vtkPolyData>::New();

  // set vertex (cell) normals
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkDoubleArray> line_normals = vtkSmartPointer<vtkDoubleArray>::New();
  line_normals->SetNumberOfComponents(3);  // 3d normals (ie x,y,z)
  line_normals->SetNumberOfTuples(static_cast<long>(tool_path_segment.size()));
  vtkSmartPointer<vtkDoubleArray> der_normals = vtkSmartPointer<vtkDoubleArray>::New();
  der_normals->SetNumberOfComponents(3);  // 3d normals (ie x,y,z)
  der_normals->SetNumberOfTuples(static_cast<long>(tool_path_segment.size()));

  int idx = 0;
  for (auto& pose : tool_path_segment)
  {
    Vector3d point, vx, vy, vz;
    point = pose.translation();
    vx = pose.linear().col(0);
    vx *= -1.0;
    vy = pose.linear().col(1);
    vz = pose.linear().col(2);
    points->InsertNextPoint(point.data());
    line_normals->SetTuple(idx, vz.data());
    der_normals->SetTuple(idx, vx.data());

    idx++;
  }
  tool_path_segment_data.line->SetPoints(points);
  tool_path_segment_data.line->GetPointData()->SetNormals(line_normals);
  tool_path_segment_data.derivatives->GetPointData()->SetNormals(der_normals);
  tool_path_segment_data.derivatives->SetPoints(points);

  return tool_path_segment_data;
}

tool_path_planner::ToolPathData toToolPathData(const tool_path_planner::ToolPath& tool_path)
{
  using namespace Eigen;

  tool_path_planner::ToolPathData tool_path_data;
  for (const auto& tool_path_segment : tool_path)
  {
    tool_path_planner::ToolPathSegmentData tool_path_segment_data = toToolPathSegmentData(tool_path_segment);
    tool_path_data.push_back(tool_path_segment_data);
  }

  return tool_path_data;
}

tool_path_planner::ToolPathsData toToolPathsData(const tool_path_planner::ToolPaths& tool_paths)
{
  using namespace Eigen;

  tool_path_planner::ToolPathsData tool_paths_data;
  for (const auto& tool_path : tool_paths)
  {
    tool_path_planner::ToolPathData tool_path_data = toToolPathData(tool_path);
    tool_paths_data.push_back(tool_path_data);
  }

  return tool_paths_data;
}

Eigen::Matrix3d toRotationMatrix(const Eigen::Vector3d& vx, const Eigen::Vector3d& vy, const Eigen::Vector3d& vz)
{
  using namespace Eigen;
  Matrix3d rot;
  rot.block(0, 0, 1, 3) = Vector3d(vx.x(), vy.x(), vz.x()).array().transpose();
  rot.block(1, 0, 1, 3) = Vector3d(vx.y(), vy.y(), vz.y()).array().transpose();
  rot.block(2, 0, 1, 3) = Vector3d(vx.z(), vy.z(), vz.z()).array().transpose();
  return rot;
}

bool toHalfedgeConfigMsg(noether_msgs::HalfedgeEdgeGeneratorConfig& config_msg,
                         const HalfedgeEdgeGenerator::Config& config)
{
  config_msg.min_num_points = config.min_num_points;
  config_msg.normal_averaging = config.normal_averaging;
  config_msg.normal_search_radius = config.normal_search_radius;
  config_msg.normal_influence_weight = config.normal_influence_weight;
  config_msg.point_spacing_method = static_cast<int>(config.point_spacing_method);
  config_msg.point_dist = config.point_dist;
  config_msg.min_allowed_obstacle_width = config.min_allowed_obstacle_width;
  config_msg.min_skip_amount = config.min_skip_amount;
  return true;
}

bool toEigenValueConfigMsg(noether_msgs::EigenValueEdgeGeneratorConfig& config_msg,
                           const EigenValueEdgeGenerator::Config& config)
{
  config_msg.octree_res = config.octree_res;
  config_msg.search_radius = config.search_radius;
  config_msg.num_threads = config.num_threads;
  config_msg.neighbor_tol = config.neighbor_tol;
  config_msg.edge_cluster_min = config.edge_cluster_min;
  config_msg.kdtree_epsilon = config.kdtree_epsilon;
  config_msg.min_projection_dist = config.min_projection_dist;
  config_msg.max_intersecting_voxels = config.max_intersecting_voxels;
  config_msg.merge_dist = config.merge_dist;
  config_msg.split_by_axes = config.split_by_axes;
  return true;
}

bool toSurfaceWalkConfigMsg(noether_msgs::SurfaceWalkRasterGeneratorConfig& config_msg,
                            const SurfaceWalkRasterGenerator::Config& config)
{
  config_msg.point_spacing = config.point_spacing;
  config_msg.raster_spacing = config.raster_spacing;
  config_msg.tool_offset = config.tool_offset;
  config_msg.intersection_plane_height = config.intersection_plane_height;
  config_msg.min_hole_size = config.min_hole_size;
  config_msg.min_segment_size = config.min_segment_size;
  config_msg.raster_rot_offset = config.raster_rot_offset;
  config_msg.generate_extra_rasters = config.generate_extra_rasters;

  config_msg.cut_direction.x = config.cut_direction[0];
  config_msg.cut_direction.y = config.cut_direction[1];
  config_msg.cut_direction.z = config.cut_direction[2];
  return true;
}

bool toPlaneSlicerConfigMsg(noether_msgs::PlaneSlicerRasterGeneratorConfig& config_msg,
                            const PlaneSlicerRasterGenerator::Config& config)
{
  config_msg.point_spacing = config.point_spacing;
  config_msg.raster_spacing = config.raster_spacing;
  config_msg.min_hole_size = config.min_hole_size;
  config_msg.min_segment_size = config.min_segment_size;
  config_msg.raster_rot_offset = config.raster_rot_offset;
  config_msg.search_radius = config.search_radius;
  config_msg.interleave_rasters = config.interleave_rasters;
  config_msg.smooth_rasters = config.smooth_rasters;
  config_msg.raster_style = config.raster_style;
  config_msg.generate_extra_rasters = config.generate_extra_rasters;
  config_msg.raster_wrt_global_axes = config.raster_wrt_global_axes;
  config_msg.raster_direction.x = config.raster_direction.x();
  config_msg.raster_direction.y = config.raster_direction.y();
  config_msg.raster_direction.z = config.raster_direction.z();

  return true;
}

bool toHalfedgeConfig(HalfedgeEdgeGenerator::Config& config,
                      const noether_msgs::HalfedgeEdgeGeneratorConfig& config_msg)
{
  config.min_num_points = config_msg.min_num_points;
  config.normal_averaging = config_msg.normal_averaging;
  config.normal_search_radius = config_msg.normal_search_radius;
  config.normal_influence_weight = config_msg.normal_influence_weight;
  config.point_spacing_method =
      static_cast<tool_path_planner::HalfedgeEdgeGenerator::PointSpacingMethod>(config_msg.point_spacing_method);
  config.point_dist = config_msg.point_dist;
  config.min_allowed_obstacle_width = config_msg.min_allowed_obstacle_width;
  config.min_skip_amount = config_msg.min_skip_amount;
  return true;
}

bool toEigenValueConfig(EigenValueEdgeGenerator::Config& config,
                        const noether_msgs::EigenValueEdgeGeneratorConfig& config_msg)
{
  config.octree_res = config_msg.octree_res;
  config.search_radius = config_msg.search_radius;
  config.num_threads = config_msg.num_threads;
  config.neighbor_tol = config_msg.neighbor_tol;
  config.edge_cluster_min = config_msg.edge_cluster_min;
  config.kdtree_epsilon = config_msg.kdtree_epsilon;
  config.min_projection_dist = config_msg.min_projection_dist;
  config.max_intersecting_voxels = config_msg.max_intersecting_voxels;
  config.merge_dist = config_msg.merge_dist;
  return true;
}

bool toSurfaceWalkConfig(SurfaceWalkRasterGenerator::Config& config,
                         const noether_msgs::SurfaceWalkRasterGeneratorConfig& config_msg)
{
  config.point_spacing = config_msg.point_spacing;
  config.raster_spacing = config_msg.raster_spacing;
  config.tool_offset = config_msg.tool_offset;
  config.intersection_plane_height = config_msg.intersection_plane_height;
  config.min_hole_size = config_msg.min_hole_size;
  config.min_segment_size = config_msg.min_segment_size;
  config.raster_rot_offset = config_msg.raster_rot_offset;
  config.generate_extra_rasters = config_msg.generate_extra_rasters;
  config.cut_direction[0] = config_msg.cut_direction.x;
  config.cut_direction[1] = config_msg.cut_direction.y;
  config.cut_direction[2] = config_msg.cut_direction.z;

  return true;
}

bool toPlaneSlicerConfig(PlaneSlicerRasterGenerator::Config& config,
                         const noether_msgs::PlaneSlicerRasterGeneratorConfig& config_msg)
{
  config.point_spacing = config_msg.point_spacing;
  config.raster_spacing = config_msg.raster_spacing;
  config.min_hole_size = config_msg.min_hole_size;
  config.min_segment_size = config_msg.min_segment_size;
  config.raster_rot_offset = config_msg.raster_rot_offset;
  config.search_radius = config_msg.search_radius;
  config.interleave_rasters = config_msg.interleave_rasters;
  config.smooth_rasters = config_msg.smooth_rasters;
  config.raster_style = static_cast<tool_path_planner::RasterStyle>(config_msg.raster_style);
  config.generate_extra_rasters = config_msg.generate_extra_rasters;
  config.raster_wrt_global_axes = config_msg.raster_wrt_global_axes;

  // Check that the raster direction was set; we are not interested in direction [0,0,0]
  Eigen::Vector3d test_raster_direction;
  test_raster_direction.x() = config_msg.raster_direction.x;
  test_raster_direction.y() = config_msg.raster_direction.y;
  test_raster_direction.z() = config_msg.raster_direction.z;
  if (!test_raster_direction.isApprox(Eigen::Vector3d::Zero()))
  {
    config.raster_direction = test_raster_direction;
  }

  return true;
}

bool createToolPathSegment(const pcl::PointCloud<pcl::PointNormal>& cloud_normals,
                           const std::vector<int>& indices,
                           ToolPathSegment& segment)
{
  using namespace pcl;
  using namespace Eigen;

  Isometry3d pose;
  Vector3d x_dir, z_dir, y_dir;
  std::vector<int> cloud_indices;
  if (indices.empty())
  {
    cloud_indices.reserve(cloud_normals.size());
    for (int i = 0; i < cloud_normals.points.size(); i++)
    {
      cloud_indices.push_back(i);
    }
  }
  else
  {
    cloud_indices.assign(indices.begin(), indices.end());
  }

  if (cloud_indices.size() < 3)
  {
    // TODO, covering the 2 point case is straight forward
    ROS_ERROR("A valid path must have at least 3 points");
    return false;
  }

  // compute first pose with x-axis using vector from first point to third point
  const PointNormal& p1 = cloud_normals[cloud_indices[0]];
  const PointNormal& p2 = cloud_normals[cloud_indices[2]];
  x_dir = (p2.getVector3fMap() - p1.getVector3fMap()).normalized().cast<double>();
  z_dir = Vector3d(p1.normal_x, p1.normal_y, p1.normal_z).normalized();
  x_dir = (x_dir - x_dir.dot(z_dir) * z_dir).normalized();  // remove component of x along z
  y_dir = z_dir.cross(x_dir).normalized();

  pose = Translation3d(p1.getVector3fMap().cast<double>());
  pose.matrix().block<3, 3>(0, 0) = tool_path_planner::toRotationMatrix(x_dir, y_dir, z_dir);
  segment.push_back(pose);

  for (std::size_t i = 1; i < cloud_indices.size() - 2; i++)
  {
    std::size_t idx_prev = cloud_indices[i - 1];
    std::size_t idx_current = cloud_indices[i];
    std::size_t idx_next = cloud_indices[i + 1];
    if (idx_current >= cloud_normals.size() || idx_next >= cloud_normals.size())
    {
      CONSOLE_BRIDGE_logError(
          "Invalid indices (current: %lu, next: %lu) for point cloud were passed", idx_current, idx_next);
      return false;
    }
    const PointNormal& p0 = cloud_normals[idx_prev];
    const PointNormal& p1 = cloud_normals[idx_current];
    const PointNormal& p2 = cloud_normals[idx_next];
    x_dir = (p2.getVector3fMap() - p0.getVector3fMap()).normalized().cast<double>();
    z_dir = Vector3d(p1.normal_x, p1.normal_y, p1.normal_z).normalized();
    x_dir = (x_dir - x_dir.dot(z_dir) * z_dir).normalized();  // remove component of x along z
    y_dir = z_dir.cross(x_dir).normalized();

    pose = Translation3d(p1.getVector3fMap().cast<double>());
    pose.matrix().block<3, 3>(0, 0) = tool_path_planner::toRotationMatrix(x_dir, y_dir, z_dir);
    segment.push_back(pose);
  }

  // compute last pose with last computed x-axis (corrected for change in z
  const PointNormal& p_last = cloud_normals[cloud_indices[cloud_indices.size() - 1]];
  z_dir = Vector3d(p_last.normal_x, p_last.normal_y, p_last.normal_z).normalized();
  x_dir = (x_dir - x_dir.dot(z_dir) * z_dir).normalized();
  y_dir = z_dir.cross(x_dir).normalized();
  pose = Translation3d(p_last.getVector3fMap().cast<double>());
  pose.matrix().block<3, 3>(0, 0) = tool_path_planner::toRotationMatrix(x_dir, y_dir, z_dir);
  segment.push_back(pose);

  return true;
}

// this is a helper function for splitPaths()
double getDistance(Eigen::Isometry3d t1, Eigen::Isometry3d t2)
{
  auto p1 = t1.translation();
  auto p2 = t2.translation();
  return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2) + pow(p1.z() - p2.z(), 2));
}

ToolPaths splitSegments(const ToolPaths& tool_paths, double max_segment_length)
{
  ToolPaths new_tool_paths;
  for (ToolPath tool_path : tool_paths)
  {
    ToolPath new_tool_path;
    for (auto seg : tool_path)
    {
      // calculate total segment length
      double total_segment_length = 0.0;
      for (std::size_t point_i = 0; point_i < seg.size() - 1; ++point_i)
      {
        total_segment_length += getDistance(seg[point_i], seg[point_i + 1]);
      }
      int num_cuts = int(ceil(total_segment_length / max_segment_length));
      double segment_length = total_segment_length / num_cuts;
      double dist_from_start = 0.0;
      std::size_t point_i = 1;
      for (int cut_i = 0; cut_i < num_cuts; ++cut_i)
      {
        ToolPathSegment new_seg;
        new_seg.push_back(seg[point_i - 1]);
        while (dist_from_start < segment_length * (cut_i + 1) && point_i < seg.size())
        {
          new_seg.push_back(seg[point_i]);
          dist_from_start += getDistance(seg[point_i - 1], seg[point_i]);
          point_i += 1;
        }
        new_tool_path.push_back(new_seg);
      }
    }
    new_tool_paths.push_back(new_tool_path);
  }
  return new_tool_paths;
}

ToolPaths reverseOddRasters(const ToolPaths& tool_paths, RasterStyle raster_style)
{
  ToolPaths new_tool_paths;
  bool is_odd = false;
  int q = 0;
  for (auto tool_path : tool_paths)
  {
    ToolPath new_tool_path;
    if (!is_odd)
    {
      for (auto seg : tool_path)
      {
        new_tool_path.push_back(seg);
      }
    }  // end if !is_odd
    else
    {  // reverse order of segments,
      for (long int i = tool_path.size() - 1; i >= 0; i--)
      {
        ToolPathSegment new_segment;
        for (long int j = tool_path[i].size() - 1; j >= 0; j--)
        {
          // rotate around z-axis of waypoint by 180 degrees (PI)
          Eigen::Isometry3d waypoint = tool_path[i][j];
          if (raster_style == FLIP_ORIENTATION_ON_REVERSE_STROKES)
            waypoint.rotate(Eigen::AngleAxisd(4 * atan(1), Eigen::Vector3d::UnitZ()));
          new_segment.push_back(waypoint);
        }
        new_tool_path.push_back(new_segment);
      }
    }
    new_tool_paths.push_back(new_tool_path);
    is_odd = !is_odd;
  }
  return new_tool_paths;
}

double computeOffsetSign(const ToolPathSegment& adjusted_segment, const ToolPathSegment& away_from_segment)
{
  double offset_sign = 1;
  Eigen::Isometry3d waypoint1 = adjusted_segment[0];
  Eigen::Isometry3d waypoint2 = away_from_segment[0];
  Eigen::Vector3d v = waypoint2.translation() - waypoint1.translation();
  Eigen::Matrix4d H = waypoint1.matrix();
  double dot_product = v.x() * H(0, 1) + v.y() * H(1, 1) + v.z() * H(2, 1);
  if (dot_product > 0.0)
    offset_sign = -1;
  return (offset_sign);
}

Eigen::Vector3d computePathDirection(const ToolPath& path)
{
  // TODO check for zero waypoints in either 1st or last segment and do something reasonable
  Eigen::Vector3d v;
  ToolPathSegment seg = path[0];
  Eigen::Isometry3d waypoint1 = seg[0];  // first waypoint in path
  seg = path[path.size() - 1];
  Eigen::Isometry3d waypoint2 = seg[seg.size() - 1];  // last waypoint in path
  v = waypoint2.translation() - waypoint1.translation();
  v.normalize();
  return (v);
}

double computePathDistance(const ToolPath& path, const Eigen::Isometry3d waypoint)
{
  double dist;

  return (dist);
}

bool compare_tuple(wp_tuple& first, wp_tuple& second) { return (std::get<0>(first) < std::get<0>(second)); }

ToolPaths addExtraPaths(const ToolPaths& tool_paths, double offset_distance)
{
  ToolPaths new_tool_paths;
  ToolPath first_dup_tool_path;  // this tool path mimics first tool_path, but offset by -y and in reverse order
  ToolPath temp_path = tool_paths[0];
  double offset_sign = 1.0;

  // duplicate first raster with an offset and add as first raster
  std::reverse(temp_path.begin(), temp_path.end());  // reverse the order of segments first
  for (ToolPathSegment seg : temp_path)
  {
    if (tool_paths.size() > 1)
    {
      ToolPath tp2 = tool_paths[1];
      offset_sign = computeOffsetSign(seg, tp2[0]);
    }
    ToolPathSegment new_segment;
    for (int i = 0; i < seg.size(); i++)
    {
      Eigen::Isometry3d waypoint = seg[i];
      Eigen::Matrix4d H = waypoint.matrix();
      waypoint.translation().x() += offset_sign * offset_distance * H(0, 1);
      waypoint.translation().y() += offset_sign * offset_distance * H(1, 1);
      waypoint.translation().z() += offset_sign * offset_distance * H(2, 1);
      new_segment.push_back(waypoint);
    }
    first_dup_tool_path.push_back(new_segment);
  }
  new_tool_paths.push_back(first_dup_tool_path);

  // add all the existing rasters to the new tool_path
  for (auto tool_path : tool_paths)
  {
    new_tool_paths.push_back(tool_path);
  }

  // duplicate last raster with an offset and add as last raster
  ToolPath last_dup_tool_path;  // this tool path mimics last tool_path, but offset by +y and in reverse order
  temp_path = tool_paths[tool_paths.size() - 1];
  std::reverse(temp_path.begin(), temp_path.end());  // reverse the segment order first
  for (ToolPathSegment seg : temp_path)
  {
    if (tool_paths.size() > 1)
    {
      ToolPath tp2 = tool_paths[tool_paths.size() - 2];
      offset_sign = computeOffsetSign(seg, tp2[0]);
    }

    ToolPathSegment new_segment;
    for (int i = 0; i < seg.size(); i++)
    {
      Eigen::Isometry3d waypoint = seg[i];
      Eigen::Matrix4d H = waypoint.matrix();
      waypoint.translation().x() += offset_sign * offset_distance * H(0, 1);
      waypoint.translation().y() += offset_sign * offset_distance * H(1, 1);
      waypoint.translation().z() += offset_sign * offset_distance * H(2, 1);
      new_segment.push_back(waypoint);
    }
    last_dup_tool_path.push_back(new_segment);
  }
  new_tool_paths.push_back(last_dup_tool_path);

  return new_tool_paths;
}

ToolPath splitByAxes(const ToolPathSegment& tool_path_segment)
{
  // Sanity check - tool path segment must not be empty
  if (tool_path_segment.size() == 0)
  {
    ROS_WARN_STREAM("Tool path segment 0 is empty.");
    ToolPath tp;
    tp.push_back(tool_path_segment);
    return tp;
  }

  // Get major and middle axis (we don't care about minor axis)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (Eigen::Isometry3d p: tool_path_segment)
  {
    pcl::PointXYZ point(float(p.translation().x()), float(p.translation().y()), float(p.translation().z()));
    cloud->push_back(point);
  }
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moment;
  moment.setInputCloud(cloud);
  moment.compute();
  Eigen::Vector3f major_axis, middle_axis, minor_axis;
  if(!moment.getEigenVectors(major_axis, middle_axis, minor_axis))
  {
    ROS_ERROR_STREAM("Could not compute Eigen Vectors.");
  }
  // Calculated perpendicular (and parallel to plane) axis using triple product
  Eigen::Vector3f perp_axis = major_axis.cross(middle_axis).cross(major_axis);
  // Normalize vectors
  perp_axis.normalize();
  major_axis.normalize();
  // Call base function
  return splitByAxes(tool_path_segment, major_axis, perp_axis);
}

ToolPath splitByAxes(const ToolPathSegment& tool_path_segment, const Eigen::Vector3f& axis_1, const Eigen::Vector3f& axis_2)
{
  // Sanity check - tool path segment must not be empty
  if (tool_path_segment.size() == 0)
  {
    ROS_WARN_STREAM("Tool path segment 0 is empty.");
    ToolPath tp;
    tp.push_back(tool_path_segment);
    return tp;
  }

  ToolPath new_tool_path;
  std::vector<Eigen::Vector3f> vectors = {
    -1 * axis_1 + -1 * axis_2,
    -1 * axis_1 +      axis_2,
         axis_1 +      axis_2,
         axis_1 + -1 * axis_2
  };
  // Get indices to cut tool path at
  std::set<int> cut_indices;

  Eigen::Vector3f path_center(0.0, 0.0, 0.0);
  for (std::size_t index = 0; index < tool_path_segment.size(); ++index)
  {
    Eigen::Isometry3d p = tool_path_segment[index];
    path_center[0] += p.translation().x();
    path_center[1] += p.translation().y();
    path_center[2] += p.translation().z();
  }

  path_center = path_center / tool_path_segment.size();

  for (Eigen::Vector3f vector : vectors)
  {
    float max_dot = std::numeric_limits<float>::min();
    int max_index = -1;
    for (std::size_t index = 0; index < tool_path_segment.size(); ++index)
    {
      Eigen::Isometry3d p = tool_path_segment[index];
      Eigen::Vector3f v(float(p.translation().x() - path_center[0]), float(p.translation().y() - path_center[1]), float(p.translation().z() - path_center[2]));
      float dot = vector.dot(v);
      if (dot > max_dot)
      {
        max_dot = dot;
        max_index = int(index);
      }
    }
    cut_indices.insert(max_index);
  }
  // Cut segment into smaller segments (at cut indices)
  for (std::set<int>::iterator i = cut_indices.begin(); i != cut_indices.end(); ++i)
  {
    ToolPathSegment new_tool_path_segment;
    int current_index = *i;
    int next_index;
    if (std::next(i, 1) == cut_indices.end())
    {
      next_index = *cut_indices.begin();
    }
    else
    {
      next_index = *std::next(i, 1);
    }
    while (current_index != next_index)
    {
      new_tool_path_segment.push_back(tool_path_segment[std::size_t(current_index)]);
      current_index = (current_index + 1) % int(tool_path_segment.size());
    }
    new_tool_path.push_back(new_tool_path_segment);
  }
  return new_tool_path;
}

ToolPaths splitByAxes(const ToolPaths& tool_paths)
{
  if (tool_paths.size() == 0)
  {
    ROS_WARN_STREAM("Tool paths is empty.");
    return tool_paths;
  }

  ToolPaths new_tool_paths;

  for (std::size_t tool_path_index = 0; tool_path_index < tool_paths.size(); ++tool_path_index)
  {
    ToolPath tool_path = tool_paths[tool_path_index];
    ToolPath new_tool_path;
    for (ToolPathSegment tool_path_segment : tool_path)
    {
      // Split segement into multiple sub-segments
      ToolPath tool_path_to_merge = splitByAxes(tool_path_segment);
      // Merge sub-segments with the other sub-segments (for this tool path)
      new_tool_path.insert(new_tool_path.end(), tool_path_to_merge.begin(), tool_path_to_merge.end());
    }
    new_tool_paths.push_back(new_tool_path);
  }
  return new_tool_paths;
}

ToolPaths splitByAxes(const ToolPaths& tool_paths, const Eigen::Vector3f& axis_1, const Eigen::Vector3f& axis_2)
{
  if (tool_paths.size() == 0)
  {
    ROS_WARN_STREAM("Tool paths is empty.");
    return tool_paths;
  }

  ToolPaths new_tool_paths;

  for (std::size_t tool_path_index = 0; tool_path_index < tool_paths.size(); ++tool_path_index)
  {
    ToolPath tool_path = tool_paths[tool_path_index];
    ToolPath new_tool_path;
    for (ToolPathSegment tool_path_segment : tool_path)
    {
      // Split segement into multiple sub-segments
      ToolPath tool_path_to_merge = splitByAxes(tool_path_segment, axis_1, axis_2);
      // Merge sub-segments with the other sub-segments (for this tool path)
      new_tool_path.insert(new_tool_path.end(), tool_path_to_merge.begin(), tool_path_to_merge.end());
    }
    new_tool_paths.push_back(new_tool_path);
  }
  return new_tool_paths;
}

struct largestJumpResults
{
    bool exists = false;
    std::size_t start_i = 0;
    std::size_t end_i = 0;
    std::size_t largest_jump = 0;
    std::vector<bool> jump_exists;
};

largestJumpResults getLargestJump(const ToolPathSegment& tool_path_seg, const double& min_allowed_width)
{
    largestJumpResults results;
    if (min_allowed_width == 0)
        return results;

    for (std::size_t i = 0; i < tool_path_seg.size(); i++)
    {
        std::vector<std::size_t> jumps;
        for (std::size_t j = 0; j < tool_path_seg.size(); j++)
        {
            // Compare the distance to every waypoint after the current one
            double dist = (tool_path_seg[i].translation() - tool_path_seg[j].translation()).norm();

            // If the distance is less than the minimum given then add it to the vector keeping track
            if (dist < min_allowed_width)
            {
                std::size_t forward_dist = std::min(j - i, tool_path_seg.size() + j - i);
                std::size_t backward_dist = std::min(i - j, tool_path_seg.size() + i - j);
                std::size_t min_dist = std::min(forward_dist, backward_dist);
                jumps.push_back(min_dist);
                if (min_dist > results.largest_jump)
                {
                    results.largest_jump = min_dist;
                    results.start_i = i;
                    results.end_i = j;
                }
            }
        }
        if (jumps.size() > 0)
        {
            std::sort(jumps.begin(), jumps.end());
            bool skip_exists = false;
            for (std::size_t j = 1; j < jumps.size(); j++)
            {
                if ((jumps[j] - jumps[j-1]) > 1)
                {
                    results.exists = true;
                    skip_exists = true;
                    break;
                }
            }
            results.jump_exists.push_back(skip_exists);
        }
    }
    return results;
}

std::vector<std::size_t> getSideWithLowerSkipPercentage(largestJumpResults jump_results)
{
    double count_inner_skips = 0;
    double count_inner_total = 0;
    double count_outer_skips = 0;
    double count_outer_total = 0;
    for (std::size_t i = 0; i < jump_results.jump_exists.size(); i++)
    {
        if ((i < jump_results.start_i && i < jump_results.end_i) ||
            (i > jump_results.start_i && i > jump_results.end_i))
        {
            if (jump_results.jump_exists[i])
                count_outer_skips++;
            count_outer_total++;
        }
        else if ((i > jump_results.start_i && i < jump_results.end_i) ||
                 (i < jump_results.start_i && i > jump_results.end_i))
        {
            if (jump_results.jump_exists[i])
                count_inner_skips++;
            count_inner_total++;
        }
    }
    double percent_inner = count_inner_skips / count_inner_total;
    double percent_outer = count_outer_skips / count_outer_total;

    std::vector<std::size_t> indices_to_keep;
    if (count_outer_total > count_inner_total)
    {
        for (std::size_t i = 0; i <= jump_results.start_i; i++)
            indices_to_keep.push_back(i);
        for (std::size_t i = jump_results.end_i; i < jump_results.jump_exists.size(); i++)
            indices_to_keep.push_back(i);
    }
    else
    {
        for (std::size_t i = jump_results.start_i; i <= jump_results.end_i; i++)
            indices_to_keep.push_back(i);
    }
    return indices_to_keep;
}

ToolPathSegment checkForPeninsulaOrInlet(const ToolPathSegment& tool_path_seg, const double& min_allowed_width, const std::size_t& min_skip_amount)
{
    ToolPathSegment output_segment = tool_path_seg;
    bool jumps_exist = true;
    std::size_t jumps_cleared = 0;
    while (jumps_exist)
    {
        largestJumpResults jump_results = getLargestJump(output_segment, min_allowed_width);
        if (jump_results.exists && jump_results.largest_jump >= min_skip_amount)
        {
            jumps_cleared++;
            std::vector<std::size_t> indices_to_keep = getSideWithLowerSkipPercentage(jump_results);
            ToolPathSegment temp_output_segment;
            for (auto i : indices_to_keep)
                temp_output_segment.push_back(output_segment[i]);
            output_segment.clear();
            output_segment = temp_output_segment;
        }
        else
        {
            jumps_exist = false;
        }
        if (jumps_cleared > 100)
            jumps_exist = false;
    }
    return output_segment;
}

void testAndMark(tlist_it& item1, tlist_it& item2, double point_spacing)
{
  int mark1 = std::get<2>(*item1);
  int mark2 = std::get<2>(*item2);
  //  Eigen::Isometry3d wp1 = std::get<1>(*item1);
  //  Eigen::Isometry3d wp2 = std::get<1>(*item2);
  //  Eigen::Vector3d v = wp1.translation() - wp2.translation();
  //  if (v.norm() < 0.75 * point_spacing)
  double dist = fabs(std::get<0>(*item1) - std::get<0>(*item2));
  if (dist < 0.75 * point_spacing)
  {
    if (mark1 == 1 && mark2 == 0)
    {
      std::get<2>(*item2) = -1;
    }
    else if (mark2 == 1 && mark1 == 0)
    {
      std::get<2>(*item1) = -1;
    }
  }
}

tlist pruneList(tlist& waypoint_list, double point_spacing)
{
  // mark any shifted waypoint for deletion if its got an original neighbor close by
  tlist_it first = waypoint_list.begin();
  tlist_it second = std::next(waypoint_list.begin(), 1);
  tlist_it third = std::next(waypoint_list.begin(), 2);
  testAndMark(first, second, point_spacing);
  testAndMark(first, third, point_spacing);
  testAndMark(second, third, point_spacing);
  for (tlist_it it = std::next(waypoint_list.begin(), 2); it != waypoint_list.end(); it++)
  {
    tlist_it it2 = std::next(it, -2);
    tlist_it it1 = std::next(it, -1);
    testAndMark(it2, it1, point_spacing);
    testAndMark(it2, it, point_spacing);
    testAndMark(it1, it, point_spacing);
  }

  tlist new_list;
  for (tlist_it it = waypoint_list.begin(); it != waypoint_list.end(); it++)
  {
    if (std::get<2>(*it) != -1)
    {
      new_list.push_back(*it);
    }
  }
  return (new_list);
}

ToolPath sortAndSegment(std::list<std::tuple<double, Eigen::Isometry3d, int> >& waypoint_list, double point_spacing)
{
  ToolPath new_tool_path;
  ToolPathSegment seg;

  waypoint_list.sort(compare_tuple);
  // we now have a sorted list including all points offset from adjacent rasters but marked with a 0 and originals
  // marked with a 1

  // next, we prune those marked with 0 if they are close to those marked with 1
  waypoint_list = pruneList(waypoint_list, point_spacing);

  // next, we prune those simply too close to one another
  Eigen::Isometry3d last_wp = std::get<1>(waypoint_list.front());
  double last_dot = std::get<0>(waypoint_list.front());
  seg.push_back(last_wp);
  int q = 0;
  for (wp_tuple waypoint_tuple : waypoint_list)
  {
    Eigen::Isometry3d waypoint = std::get<1>(waypoint_tuple);
    int mark = std::get<2>(waypoint_tuple);
    Eigen::Vector3d v = waypoint.translation() - last_wp.translation();
    double cart_spacing = v.norm();
    double dot_spacing = std::get<0>(waypoint_tuple) - last_dot;

    // complex if statement
    // spacing computed using dot-distance is close to the point spacing AND
    // spacing computed using cartesian distance is close to point spacing
    // then add add to segment
    if (dot_spacing > .7 * point_spacing && dot_spacing <= 1.3 * point_spacing && cart_spacing > .7 * point_spacing &&
            cart_spacing <= 1.3 * point_spacing ||
        (mark == 1 && dot_spacing > 0.0 && cart_spacing < 1.3 * point_spacing))
    {
      seg.push_back(waypoint);
      last_wp = waypoint;
      last_dot = std::get<0>(waypoint_tuple);
    }
    else if (dot_spacing > 1.3 * point_spacing && cart_spacing > 1.3 * point_spacing)  // only add extra if dot spacing
                                                                                       // is large
    {                                                                                  // start a new segment
      if (seg.size() > 3)
      {
        new_tool_path.push_back(seg);  // throw away very short segments
      }
      seg.clear();
      seg.push_back(waypoint);
      last_wp = waypoint;
      last_dot = std::get<0>(waypoint_tuple);
    }
    else
    {  // skip unless last in list, significantly spaced from last waypoint, but not too far
      if (q == waypoint_list.size() - 1 && dot_spacing > .3 * point_spacing && cart_spacing < 1.3 * point_spacing)
      {
        // seg.push_back(waypoint);  // keep the last on regardless of distance to proces up to the defined edges
        last_dot = std::get<0>(waypoint_tuple);
      }
    }
    q++;
  }  // end for every waypoint in waypoint_list
  if (seg.size() > 3)
  {
    new_tool_path.push_back(seg);
  }
  return (new_tool_path);
}

ToolPaths addExtraWaypoints(const ToolPaths& tool_paths, double raster_spacing, double point_spacing)
{
  ToolPaths new_tool_paths;
  double offset_sign = 1;

  if (tool_paths.size() > 1)
  {
    ToolPath tool_path1 = tool_paths[0];
    ToolPath tool_path2 = tool_paths[1];
    offset_sign = computeOffsetSign(tool_path1[0], tool_path2[0]);
  }

  for (size_t i = 0; i < tool_paths.size(); i++)
  {
    // create a list of tuple containing the waypoints in this tool_path and its distance along the path
    // add to this list the waypoints and distances of the previous and next path offset into this path
    // sort the list
    // using sorted list, create new_tool_path by:
    // add a new point if it is approximately point_spacing from the previous point
    // end the old and start a new segment when the new point is significantly more than the point_spacing from previous
    // waypoint
    ToolPath tool_path = tool_paths[i];
    std::list<std::tuple<double, Eigen::Isometry3d, int> > waypoint_list;  // once sorted this list will be the order of
                                                                           // the segments
    Eigen::Vector3d path_dir = computePathDirection(tool_path);
    ToolPathSegment sseg = tool_path[0];
    Eigen::Vector3d path_start = sseg[0].translation();

    if (i == 0)  // duplicate first tool_path but offset
    {
      for (ToolPathSegment seg : tool_path)  // add tool_path's waypoints to the list
      {
        for (Eigen::Isometry3d waypoint : seg)
        {
          Eigen::Matrix4d H = waypoint.matrix();
          waypoint.translation().x() += offset_sign * raster_spacing * H(0, 1);
          waypoint.translation().y() += offset_sign * raster_spacing * H(1, 1);
          waypoint.translation().z() += offset_sign * raster_spacing * H(2, 1);
          Eigen::Vector3d v = waypoint.translation() - path_start;
          std::tuple<double, Eigen::Isometry3d, int> p(path_dir.dot(v), waypoint, 0);
          waypoint_list.push_back(p);
        }  // end for every waypoint in segment
      }    // end for every segment in path
      new_tool_paths.push_back(sortAndSegment(waypoint_list, point_spacing));
      waypoint_list.clear();
    }  // end duplication of fist tool_path but offset

    for (ToolPathSegment seg : tool_path)  // add tool_path's waypoints to the list
    {
      for (Eigen::Isometry3d waypoint : seg)
      {
        Eigen::Vector3d v = waypoint.translation() - path_start;
        std::tuple<double, Eigen::Isometry3d, int> p(path_dir.dot(v), waypoint, 1);
        waypoint_list.push_back(p);
      }  // end for every waypoint in segment
    }    // end for every segment in path
    if (i > 0)
    {
      ToolPath prev_tool_path = tool_paths[i - 1];
      for (ToolPathSegment seg : prev_tool_path)  // add previous tool_path's waypoints to the list
      {
        for (Eigen::Isometry3d waypoint : seg)
        {
          Eigen::Matrix4d H = waypoint.matrix();
          waypoint.translation().x() -= offset_sign * raster_spacing * H(0, 1);
          waypoint.translation().y() -= offset_sign * raster_spacing * H(1, 1);
          waypoint.translation().z() -= offset_sign * raster_spacing * H(2, 1);
          Eigen::Vector3d v = waypoint.translation() - path_start;
          std::tuple<double, Eigen::Isometry3d, int> p(path_dir.dot(v), waypoint, 0);
          waypoint_list.push_back(p);
        }  // end for every waypoint in segment
      }    // end for every segment in path
    }
    if (i < tool_paths.size() - 1)
    {
      ToolPath next_tool_path = tool_paths[i + 1];
      for (ToolPathSegment seg : next_tool_path)  // add next tool_path's waypoints to the list
      {
        for (Eigen::Isometry3d waypoint : seg)
        {
          Eigen::Matrix4d H = waypoint.matrix();
          waypoint.translation().x() += offset_sign * raster_spacing * H(0, 1);
          waypoint.translation().y() += offset_sign * raster_spacing * H(1, 1);
          waypoint.translation().z() += offset_sign * raster_spacing * H(2, 1);
          Eigen::Vector3d v = waypoint.translation() - path_start;
          std::tuple<double, Eigen::Isometry3d, int> p(path_dir.dot(v), waypoint, 0);
          waypoint_list.push_back(p);
        }  // end for every waypoint in segment
      }    // end for every segment in path
    }
    new_tool_paths.push_back(sortAndSegment(waypoint_list, point_spacing));
    waypoint_list.clear();
    // duplicate last tool_path but offset
    if (i == tool_paths.size() - 1)
    {
      for (ToolPathSegment seg : tool_path)  // add tool_path's waypoints to the list
      {
        for (Eigen::Isometry3d waypoint : seg)
        {
          Eigen::Matrix4d H = waypoint.matrix();
          waypoint.translation().x() -= offset_sign * raster_spacing * H(0, 1);
          waypoint.translation().y() -= offset_sign * raster_spacing * H(1, 1);
          waypoint.translation().z() -= offset_sign * raster_spacing * H(2, 1);
          Eigen::Vector3d v = waypoint.translation() - path_start;
          std::tuple<double, Eigen::Isometry3d, int> p(path_dir.dot(v), waypoint, 1);
          waypoint_list.push_back(p);
        }  // end for every waypoint in segment

      }  // end for every segment in path
      new_tool_paths.push_back(sortAndSegment(waypoint_list, point_spacing));
    }  // end duplication of last tool_path but offset
  }    // end for every path in tool_paths
  return new_tool_paths;
}

// computes the normal vectors to a mesh at every vertex
pcl::PointCloud<pcl::PointNormal> computeMLSMeshNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& mesh_cloud,
                                                        double normal_search_radius)
{
  // initialize the MLSNE object
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> MLS;
  MLS.setInputCloud(mesh_cloud);
  MLS.setComputeNormals(true);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  MLS.setSearchMethod(tree);
  MLS.setPolynomialOrder(3);
  MLS.setSearchRadius(normal_search_radius);
  MLS.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod::NONE);
  MLS.setCacheMLSResults(true);
  MLS.setProjectionMethod(pcl::MLSResult::ProjectionMethod::ORTHOGONAL);
  // NONE, DISTINCT_CLOUD, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION
  // unused initialization methods here to help future developers
  // MLS.setUpsamplingRadius(), MLS.setUpsamplingStepSize(), MLS.setDistinctCloud()

  // process the cloud
  pcl::PointCloud<pcl::PointNormal> mls_mesh_cloud_normals;
  MLS.process(mls_mesh_cloud_normals);
  std::vector<pcl::MLSResult> R = MLS.getMLSResults();

  pcl::PointCloud<pcl::PointNormal> mesh_cloud_normals;
  mesh_cloud_normals.reserve(mesh_cloud->points.size());
  for (size_t i = 0; i < mesh_cloud->points.size(); i++)
  {
    pcl::PointNormal new_pn;
    new_pn.x = mesh_cloud->points[i].x;
    new_pn.y = mesh_cloud->points[i].y;
    new_pn.z = mesh_cloud->points[i].z;
    new_pn.normal_x = -R[i].plane_normal.x();
    new_pn.normal_y = -R[i].plane_normal.y();
    new_pn.normal_z = -R[i].plane_normal.z();
    mesh_cloud_normals.push_back(new_pn);
  }
  return (mesh_cloud_normals);
}

/**
 * @brief forces the required point spacing by computing new points along the curve
 * @param in    The input cloud
 * @param out   The output cloud
 * @param dist  The required point spacing distance
 * @return True on success, False otherwise
 */
bool applyEqualDistance(const std::vector<int>& pnt_indices,
                        const pcl::PointCloud<pcl::PointXYZ>& mesh_cloud,
                        pcl::PointCloud<pcl::PointNormal>& path_cloud,
                        double dist)
{
  using namespace pcl;
  using namespace Eigen;

  PointNormal new_pt;
  PointXYZ p_start, p_mid, p_end;
  Vector3f v_1, v_2, unitv_2, v_next;
  if (pnt_indices.size() < 3)
  {
    CONSOLE_BRIDGE_logError("pnt_indices must have at least 3");
    return false;
  }
  PointXYZ pt = mesh_cloud.points[pnt_indices[0]];
  p_start.x = pt.x;
  p_start.y = pt.y;
  p_start.z = pt.z;
  pt = mesh_cloud.points[pnt_indices[1]];
  p_mid.x = pt.x;
  p_mid.y = pt.y;
  p_mid.z = pt.z;
  v_1 = p_mid.getVector3fMap() - p_start.getVector3fMap();
  new_pt.x = mesh_cloud[pnt_indices[0]].x;
  new_pt.y = mesh_cloud[pnt_indices[0]].y;
  new_pt.z = mesh_cloud[pnt_indices[0]].z;
  path_cloud.push_back(new_pt);  // add first point
  for (std::size_t i = 2; i < pnt_indices.size(); i++)
  {
    pt = mesh_cloud[pnt_indices[i]];
    p_end.x = pt.x;
    p_end.y = pt.y;
    p_end.z = pt.z;
    while (dist < v_1.norm())
    {
      p_start.getVector3fMap() = p_start.getVector3fMap() + dist * v_1.normalized();
      std::tie(new_pt.x, new_pt.y, new_pt.z) = std::make_tuple(p_start.x, p_start.y, p_start.z);
      path_cloud.push_back(new_pt);
      v_1 = p_mid.getVector3fMap() - p_start.getVector3fMap();
    }

    v_2 = p_end.getVector3fMap() - p_mid.getVector3fMap();
    if (dist < (v_1 + v_2).norm())
    {
      // solve for x such that " x^2 + 2 * x * dot(v_1, unitv_2) + norm(v_1)^2 - d^2 = 0"
      unitv_2 = v_2.normalized();
      double b = 2 * v_1.dot(unitv_2);
      double c = std::pow(v_1.norm(), 2) - std::pow(dist, 2);
      double sqrt_term = std::sqrt(std::pow(b, 2) - 4 * c);
      double x = 0.5 * (-b + sqrt_term);
      x = (x > 0.0 && x < dist) ? x : 0.5 * (-b - sqrt_term);
      if (x > dist)
      {
        return false;
      }
      v_next = v_1 + x * unitv_2;

      // computing next point at distance "dist" from previous that lies on the curve
      p_start.getVector3fMap() = p_start.getVector3fMap() + v_next;
      std::tie(new_pt.x, new_pt.y, new_pt.z) = std::make_tuple(p_start.x, p_start.y, p_start.z);
      path_cloud.push_back(new_pt);
    }

    // computing values for next iter
    p_mid = p_end;
    v_1 = p_mid.getVector3fMap() - p_start.getVector3fMap();
  }

  // add last point if not already there
  if ((path_cloud.back().getVector3fMap() - mesh_cloud[pnt_indices.back()].getVector3fMap()).norm() > dist / 3.0)
  {
    new_pt.x = mesh_cloud[pnt_indices[pnt_indices.size() - 1]].x;
    new_pt.y = mesh_cloud[pnt_indices[pnt_indices.size() - 1]].y;
    new_pt.z = mesh_cloud[pnt_indices[pnt_indices.size() - 1]].z;
    path_cloud.push_back(new_pt);
  }

  if (path_cloud.size() < 2)
  {
    CONSOLE_BRIDGE_logError("Points in curve segment are too close together");
    return false;
  }

  return true;
}

bool insertPointNormals(const pcl::PointCloud<pcl::PointNormal>::Ptr& mesh_cloud,
                        pcl::PointCloud<pcl::PointNormal>& path_cloud)
{
  // recovering normals using kd-treek
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_points(new pcl::PointCloud<pcl::PointXYZ>());
  for (pcl::PointNormal p : mesh_cloud->points)
  {
    pcl::PointXYZ pt(p.x, p.y, p.z);
    in_points->points.push_back(pt);
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(in_points);
  kdtree.setEpsilon(EPSILON);

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;
  pcl::PointXYZ query_point;
  for (auto& p : path_cloud)
  {
    k_indices.resize(1);
    k_sqr_distances.resize(1);
    std::tie(query_point.x, query_point.y, query_point.z) = std::make_tuple(p.x, p.y, p.z);
    if (kdtree.nearestKSearch(query_point, 1, k_indices, k_sqr_distances) < 1)
    {
      CONSOLE_BRIDGE_logError("No nearest points found");
      return false;
    }

    int idx = k_indices[0];
    if (idx < mesh_cloud->points.size())
    {
      std::tie(p.normal_x, p.normal_y, p.normal_z) = std::make_tuple(
          mesh_cloud->points[idx].normal_x, mesh_cloud->points[idx].normal_y, mesh_cloud->points[idx].normal_z);
    }
    else
    {
      ROS_ERROR("what is wrong %d", idx);
    }
  }

  return true;
}

// TODO, why is this here when noether_conversions::convertToPCLMesh(const shape_msgs::Mesh& mesh_msg, pcl::PolygonMesh&
// mesh) returns a pcl::PointCloud<> with mesh.cloud being of type pcl::PointCloud<pcl::PointXYZ>
void shapeMsgToPclPointXYZ(const shape_msgs::Mesh& mesh, pcl::PointCloud<pcl::PointXYZ>& mesh_cloud)
{
  mesh_cloud.points.clear();
  for (int i = 0; i < mesh.vertices.size(); i++)
  {
    pcl::PointXYZ p(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);
    mesh_cloud.points.push_back(p);
  }
}

void computeFaceNormals(const shape_msgs::Mesh& mesh, std::vector<Eigen::Vector3d>& face_normals)
{
  face_normals.clear();
  face_normals.reserve(mesh.triangles.size());
  for (size_t i = 0; i < mesh.triangles.size(); i++)
  {
    size_t idx1 = mesh.triangles[i].vertex_indices[0];
    size_t idx2 = mesh.triangles[i].vertex_indices[1];
    size_t idx3 = mesh.triangles[i].vertex_indices[2];
    Eigen::Vector3d v1(mesh.vertices[idx1].x, mesh.vertices[idx1].y, mesh.vertices[idx1].z);
    Eigen::Vector3d v2(mesh.vertices[idx2].x, mesh.vertices[idx2].y, mesh.vertices[idx2].z);
    Eigen::Vector3d v3(mesh.vertices[idx3].x, mesh.vertices[idx3].y, mesh.vertices[idx3].z);
    Eigen::Vector3d normal = v1.cross(v2) + v2.cross(v3) + v3.cross(v1);
    normal.normalize();
    face_normals.push_back(normal);
  }
}

void computeFaceNormals(pcl::PolygonMesh::ConstPtr& mesh, std::vector<Eigen::Vector3d>& face_normals)
{
  face_normals.clear();
  face_normals.reserve(mesh->polygons.size());
  for (size_t i = 0; i < mesh->polygons.size(); i++)
  {
    size_t idx1 = mesh->polygons[i].vertices[0] * mesh->cloud.point_step;
    size_t idx2 = mesh->polygons[i].vertices[1] * mesh->cloud.point_step;
    size_t idx3 = mesh->polygons[i].vertices[2] * mesh->cloud.point_step;
    size_t xo = mesh->cloud.fields[0].offset;
    size_t yo = mesh->cloud.fields[1].offset;
    size_t zo = mesh->cloud.fields[2].offset;
    float* x = (float*)(&mesh->cloud.data[idx1 + xo]);
    float* y = (float*)(&mesh->cloud.data[idx1 + yo]);
    float* z = (float*)(&mesh->cloud.data[idx1 + zo]);
    Eigen::Vector3d v1(*x, *y, *z);
    x = (float*)(&mesh->cloud.data[idx2 + xo]);
    y = (float*)(&mesh->cloud.data[idx2 + yo]);
    z = (float*)(&mesh->cloud.data[idx2 + zo]);
    Eigen::Vector3d v2(*x, *y, *z);
    x = (float*)(&mesh->cloud.data[idx3 + xo]);
    y = (float*)(&mesh->cloud.data[idx3 + yo]);
    z = (float*)(&mesh->cloud.data[idx3 + zo]);
    Eigen::Vector3d v3(*x, *y, *z);
    Eigen::Vector3d normal = v1.cross(v2) + v2.cross(v3) + v3.cross(v1);
    normal.normalize();
    face_normals.push_back(normal);
  }
}

void computeMeshNormals(const shape_msgs::Mesh& mesh,
                        std::vector<Eigen::Vector3d>& face_normals,
                        std::vector<Eigen::Vector3d>& vertex_normals)
{
  computeFaceNormals(mesh, face_normals);

  // populate a structure for each vertex containing a list of adjoining faces
  std::vector<size_t> vertex_faces[mesh.vertices.size()];
  for (size_t i = 0; i < mesh.triangles.size(); i++)  // find every face adjoining each vertex
  {
    vertex_faces[mesh.triangles[i].vertex_indices[0]].push_back(i);
    vertex_faces[mesh.triangles[i].vertex_indices[1]].push_back(i);
    vertex_faces[mesh.triangles[i].vertex_indices[2]].push_back(i);
  }
  // average the adjoining face normals
  vertex_normals.clear();
  vertex_normals.reserve(mesh.vertices.size());
  for (size_t i = 0; i < mesh.vertices.size(); i++)  // for each vertex averages all adjoining face normals
  {
    Eigen::Vector3d vertex_normal;
    if (vertex_faces[i].size() == 0)
    {
      vertex_normal.z() = 1;
    }
    else
    {
      for (size_t j = 0; j < vertex_faces[i].size(); j++)
      {
        size_t face_index = vertex_faces[i][j];
        vertex_normal += face_normals[face_index];
      }
    }
    vertex_normal.normalize();
    vertex_normals.push_back(vertex_normal);
  }
}

void computePCLMeshNormals(pcl::PolygonMesh::ConstPtr& mesh,
                           std::vector<Eigen::Vector3d>& face_normals,
                           std::vector<Eigen::Vector3d>& vertex_normals)
{
  computeFaceNormals(mesh, face_normals);
  int num_pts = mesh->cloud.height * mesh->cloud.width;

  // populate a structure for each vertex containing a list of adjoining faces
  std::vector<size_t> vertex_faces[num_pts];
  for (size_t i = 0; i < mesh->polygons.size(); i++)  // find every face adjoining each vertex
  {
    vertex_faces[mesh->polygons[i].vertices[0]].push_back(i);
    vertex_faces[mesh->polygons[i].vertices[1]].push_back(i);
    vertex_faces[mesh->polygons[i].vertices[2]].push_back(i);
  }
  // average the adjoining face normals
  vertex_normals.clear();
  vertex_normals.reserve(num_pts);
  for (size_t i = -0; i < num_pts; i++)  // for each vertex averages all adjoining face normals
  {
    Eigen::Vector3d vertex_normal;
    if (vertex_faces[i].size() == 0)
    {
      vertex_normal.z() = 1;
    }
    else
    {
      for (size_t j = 0; j < vertex_faces[i].size(); j++)
      {
        size_t face_index = vertex_faces[i][j];
        vertex_normal += face_normals[face_index];
      }
    }
    vertex_normal.normalize();
    vertex_normals.push_back(vertex_normal);
  }
}

bool alignToVertexNormals(pcl::PointCloud<pcl::PointNormal>& mesh_cloud, std::vector<Eigen::Vector3d>& vertex_normals)
{
  if (mesh_cloud.points.size() != vertex_normals.size())
  {
    ROS_ERROR("mesh_cloud and vertex_normals have different number of points %ld vs %ld",
              mesh_cloud.points.size(),
              vertex_normals.size());
    return (false);
  }
  for (size_t i = 0; i < mesh_cloud.points.size(); i++)
  {
    Eigen::Vector3d n(mesh_cloud.points[i].normal_x, mesh_cloud.points[i].normal_y, mesh_cloud.points[i].normal_z);
    if (vertex_normals[i].dot(n) < 0)
    {
      mesh_cloud.points[i].normal_x = -mesh_cloud.points[i].normal_x;
      mesh_cloud.points[i].normal_y = -mesh_cloud.points[i].normal_y;
      mesh_cloud.points[i].normal_z = -mesh_cloud.points[i].normal_z;
    }
  }
  return true;
}
}  // namespace tool_path_planner
