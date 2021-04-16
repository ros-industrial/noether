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

namespace tool_path_planner
{
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

tool_path_planner::ToolPathsData toToolPathsData(const tool_path_planner::ToolPaths& paths)
{
  using namespace Eigen;

  tool_path_planner::ToolPathsData results;
  for (const auto& path : paths)
  {
    tool_path_planner::ToolPathData tp_data;
    for (const auto& segment : path)
    {
      tool_path_planner::ToolPathSegmentData tps_data;
      tps_data.line = vtkSmartPointer<vtkPolyData>::New();
      tps_data.derivatives = vtkSmartPointer<vtkPolyData>::New();

      // set vertex (cell) normals
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
      vtkSmartPointer<vtkDoubleArray> line_normals = vtkSmartPointer<vtkDoubleArray>::New();
      line_normals->SetNumberOfComponents(3);  // 3d normals (ie x,y,z)
      line_normals->SetNumberOfTuples(static_cast<long>(segment.size()));
      vtkSmartPointer<vtkDoubleArray> der_normals = vtkSmartPointer<vtkDoubleArray>::New();
      ;
      der_normals->SetNumberOfComponents(3);  // 3d normals (ie x,y,z)
      der_normals->SetNumberOfTuples(static_cast<long>(segment.size()));

      int idx = 0;
      for (auto& pose : segment)
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
      tps_data.line->SetPoints(points);
      tps_data.line->GetPointData()->SetNormals(line_normals);
      tps_data.derivatives->GetPointData()->SetNormals(der_normals);
      tps_data.derivatives->SetPoints(points);

      tp_data.push_back(tps_data);
    }
    results.push_back(tp_data);
  }
  return results;
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
  config_msg.max_segment_length = config.max_segment_length;
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
    cloud_indices.resize(cloud_normals.size());
    std::iota(cloud_indices.begin(), cloud_indices.end(), 0);
  }
  else
  {
    cloud_indices.assign(indices.begin(), indices.end());
  }

  for (std::size_t i = 0; i < cloud_indices.size() - 1; i++)
  {
    std::size_t idx_current = cloud_indices[i];
    std::size_t idx_next = cloud_indices[i + 1];
    if (idx_current >= cloud_normals.size() || idx_next >= cloud_normals.size())
    {
      CONSOLE_BRIDGE_logError(
          "Invalid indices (current: %lu, next: %lu) for point cloud were passed", idx_current, idx_next);
      return false;
    }
    const PointNormal& p1 = cloud_normals[idx_current];
    const PointNormal& p2 = cloud_normals[idx_next];
    x_dir = (p2.getVector3fMap() - p1.getVector3fMap()).normalized().cast<double>();
    z_dir = Vector3d(p1.normal_x, p1.normal_y, p1.normal_z).normalized();
    y_dir = z_dir.cross(x_dir).normalized();

    pose = Translation3d(p1.getVector3fMap().cast<double>());
    pose.matrix().block<3, 3>(0, 0) = tool_path_planner::toRotationMatrix(x_dir, y_dir, z_dir);
    segment.push_back(pose);
  }

  // last pose
  Eigen::Isometry3d p = segment.back();
  p.translation().x() = cloud_normals[cloud_indices.back()].x;
  p.translation().y() = cloud_normals[cloud_indices.back()].y;
  p.translation().z() = cloud_normals[cloud_indices.back()].z;
  segment.push_back(p);

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

ToolPaths segmentByAxes(const ToolPaths& tool_paths, SegmentAxes::SegmentAxes segment_axes)
{
  if (tool_paths.size() == 0)
  {
    ROS_ERROR_STREAM("Tool paths is empty.");
    // TODO: throw an exception
  }

  ToolPaths new_tool_paths;

  double axes_distance = 1000.0;
  std::vector<Eigen::Vector3d> corner_points;

  Eigen::Vector3d p1, p2, p3, p4;
  if (segment_axes == SegmentAxes::XY)
  {
    p1.x() = -axes_distance;
    p1.y() = -axes_distance;
    p1.z() = 0;

    p2.x() = -axes_distance;
    p2.y() = axes_distance;
    p2.z() = 0;

    p3.x() = axes_distance;
    p3.y() = -axes_distance;
    p3.z() = 0;

    p4.x() = axes_distance;
    p4.y() = axes_distance;
    p4.z() = 0;
  }
  else if (segment_axes == SegmentAxes::XZ)
  {
    p1.x() = -axes_distance;
    p1.y() = 0;
    p1.z() = -axes_distance;

    p2.x() = -axes_distance;
    p2.y() = 0;
    p2.z() = axes_distance;

    p3.x() = axes_distance;
    p3.y() = 0;
    p3.z() = -axes_distance;

    p4.x() = axes_distance;
    p4.y() = 0;
    p4.z() = axes_distance;
  }
  else // if (segment_axes == SegmentAxes::YZ)
  {
    p1.x() = 0;
    p1.y() = -axes_distance;
    p1.z() = -axes_distance;

    p2.x() = 0;
    p2.y() = -axes_distance;
    p2.z() = axes_distance;

    p3.x() = 0;
    p3.y() = axes_distance;
    p3.z() = -axes_distance;

    p4.x() = 0;
    p4.y() = axes_distance;
    p4.z() = axes_distance;
  }

  corner_points.push_back(p1);
  corner_points.push_back(p2);
  corner_points.push_back(p3);
  corner_points.push_back(p4);

  for (std::size_t path_index = 0; path_index < tool_paths.size(); ++path_index)
  {
    ToolPath tool_path = tool_paths[path_index];
    if (tool_path.size() != 1)
    {
      ROS_ERROR_STREAM("Tool path " << path_index << " contains " << tool_path.size() <<
                       " segments. Expected exactly 1.");
      // TODO: throw an exception
    }
    ToolPath new_tool_path;
    ToolPathSegment tool_path_segment = tool_path[0];
    if (tool_path_segment.size() == 0)
    {
      ROS_ERROR_STREAM("Tool path " << path_index << " segment 0 is empty.");
      // TODO: throw an exception
    }

    // get cut indices
    std::vector<std::size_t> cut_indices;
    for (Eigen::Vector3d corner_point : corner_points)
    {
      double min_distance = axes_distance * 10;
      int min_index = -1;
      for (std::size_t point_index = 0; point_index < tool_path_segment.size(); ++ point_index)
      {
        Eigen::Vector3d projected_point = tool_path_segment[point_index].translation();
        if (segment_axes == SegmentAxes::XY)
        {
          projected_point.z() = 0;
        }
        else if (segment_axes == SegmentAxes::XZ)
        {
          projected_point.y() = 0;
        }
        else // if (segment_axes == SegmentAxes::YZ)
        {
          projected_point.x() = 0;
        }
        double distance = (projected_point - corner_point).norm();
        if (distance < min_distance)
        {
          min_distance = distance;
          min_index = int(point_index);
        }
      }
      // check if we found a closest point
      if (min_index == -1)
      {
        // if this happens, that means that all tool path points were more than (10 * axes_distance) meters from
        // the origin -- this violates the assumption that tool path points are REASONABLY close to origin
        ROS_ERROR_STREAM("Could not find closest point to corner point for path " << path_index << " segment 0 "
                         "for corner point at x:" << corner_point.x() << " y:" << corner_point.y() << " z:" <<
                         corner_point.z());
        // TODO: throw a custom exception
      }
      // check if the point was also closest to a previous corner (i.e. already in list)
      bool already_exists = false;
      for (std::size_t cut_index : cut_indices)
      {
        if (cut_index == std::size_t(min_index))
        {
          already_exists = true;
        }
      }
      // if this point isn't already in the list, add to cut indices
      if (!already_exists)
      {
        cut_indices.push_back(std::size_t(min_index));
      }
    }

    // cut tool path
    // sort cut indices
    std::sort(cut_indices.begin(), cut_indices.end());
    // iterate each cut index
    for (std::size_t i = 0; i < cut_indices.size(); ++i)
    {
      ToolPathSegment new_tool_path_segment;
      std::size_t index = cut_indices[i];
      // while index does not equal the next cut index (including going from last index to first index)
      // add each index after the current cut index, up until the next cut index
      while (index != cut_indices[(i+1)%cut_indices.size()])
      {
        new_tool_path_segment.push_back(tool_path_segment[index]);
        index = (index + 1) % tool_path_segment.size();
      }
      new_tool_path.push_back(new_tool_path_segment);
    }
    new_tool_paths.push_back(new_tool_path);
  }
  return new_tool_paths;
}

}  // namespace tool_path_planner
