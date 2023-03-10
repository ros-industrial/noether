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
  // config_msg.tool_offset = config.tool_offset;
  config_msg.min_hole_size = config.min_hole_size;
  config_msg.min_segment_size = config.min_segment_size;
  config_msg.raster_rot_offset = config.raster_rot_offset;
  config_msg.search_radius = config.search_radius;

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
  // config.tool_offset = config_msg.tool_offset;
  config.min_hole_size = config_msg.min_hole_size;
  config.min_segment_size = config_msg.min_segment_size;
  config.raster_rot_offset = config_msg.raster_rot_offset;
  config.search_radius = config_msg.search_radius;

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

}  // namespace tool_path_planner
