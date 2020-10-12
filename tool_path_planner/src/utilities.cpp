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
#include <eigen_conversions/eigen_msg.h>

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

void flipPointOrder(tool_path_planner::ProcessPath& path)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
  points = path.line->GetPoints();

  // flip point order
  for(int i = points->GetNumberOfPoints() - 1; i >= 0; --i)
  {
    points2->InsertNextPoint(points->GetPoint(i));
  }
  path.line->SetPoints(points2);

  // flip normal order
  vtkSmartPointer<vtkDataArray> norms = path.line->GetPointData()->GetNormals();
  vtkSmartPointer<vtkDoubleArray> new_norms = vtkSmartPointer<vtkDoubleArray>::New();
  new_norms->SetNumberOfComponents(3);

  for(int i = norms->GetNumberOfTuples() - 1; i >= 0; --i)
  {
    double* ptr = norms->GetTuple(i);
    new_norms->InsertNextTuple(ptr);
  }
  path.line->GetPointData()->SetNormals(new_norms);

  // flip point order
  points = path.derivatives->GetPoints();
  vtkSmartPointer<vtkPoints> dpoints2 = vtkSmartPointer<vtkPoints>::New();
  for(int i = points->GetNumberOfPoints() - 1; i >= 0; --i)
  {
    dpoints2->InsertNextPoint(points->GetPoint(i));
  }
  path.derivatives->SetPoints(dpoints2);

  // flip derivative directions
  vtkDataArray* ders = path.derivatives->GetPointData()->GetNormals();
  vtkSmartPointer<vtkDoubleArray> new_ders = vtkSmartPointer<vtkDoubleArray>::New();
  new_ders->SetNumberOfComponents(3);
  for(int i = ders->GetNumberOfTuples() -1; i >= 0; --i)
  {
    double* pt = ders->GetTuple(i);
    pt[0] *= -1;
    pt[1] *= -1;
    pt[2] *= -1;
    new_ders->InsertNextTuple(pt);
  }
  path.derivatives->GetPointData()->SetNormals(new_ders);

  // reset points in spline
  path.spline->SetPoints(points);
}

std::vector<geometry_msgs::PoseArray> convertVTKtoGeometryMsgs(
    const std::vector<tool_path_planner::ProcessPath>& paths)
{
  std::vector<geometry_msgs::PoseArray> poseArrayVector;
  for(int j = 0; j < paths.size(); ++j)
  {
     geometry_msgs::PoseArray poses;
     poses.header.seq = j;
     poses.header.stamp = ros::Time::now();
     poses.header.frame_id = "0";
     for(int k = 0; k < paths[j].line->GetPoints()->GetNumberOfPoints(); ++k)
     {
        geometry_msgs::Pose pose;
        double pt[3];

        // Get the point location
        paths[j].line->GetPoints()->GetPoint(k, pt);
        pose.position.x = pt[0];
        pose.position.y = pt[1];
        pose.position.z = pt[2];

        // Get the point normal and derivative for creating the 3x3 transform
        double* norm =
            paths[j].line->GetPointData()->GetNormals()->GetTuple(k);
        double* der =
            paths[j].derivatives->GetPointData()->GetNormals()->GetTuple(k);

        // perform cross product to get the third axis direction
        Eigen::Vector3d u(norm[0], norm[1], norm[2]);
        Eigen::Vector3d v(der[0], der[1], der[2]);
        Eigen::Vector3d w = u.cross(v);
        w.normalize();

        // after first cross product, u and w will be orthogonal.
        // Perform cross product one more time to make sure that v is perfectly
        // orthogonal to u and w
        v = u.cross(w);
        v.normalize();

        Eigen::Affine3d epose = Eigen::Affine3d::Identity();
        epose.matrix().col(0).head<3>() = v;
        epose.matrix().col(1).head<3>() = -w;
        epose.matrix().col(2).head<3>() = u;
        epose.matrix().col(3).head<3>() = Eigen::Vector3d(pt[0], pt[1], pt[2]);

        tf::poseEigenToMsg(epose, pose);

        // push back new matrix (pose and orientation), this makes one long
        // vector may need to break this up more
        poses.poses.push_back(pose);

      }
      poseArrayVector.push_back(poses);
    }

  return poseArrayVector;
}

std::vector<tool_path_planner::ProcessPath> toNoetherToolpaths(const noether_msgs::ToolRasterPath& paths)
{
  using namespace Eigen;
  std::vector<tool_path_planner::ProcessPath> noether_tp;

  for(auto& s : paths.paths)
  {
    tool_path_planner::ProcessPath tpp;
    tpp.line = vtkSmartPointer<vtkPolyData>::New();
    tpp.derivatives = vtkSmartPointer<vtkPolyData>::New();

    //set vertex (cell) normals
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkDoubleArray> line_normals = vtkSmartPointer<vtkDoubleArray>::New();
    line_normals->SetNumberOfComponents(3); //3d normals (ie x,y,z)
    line_normals->SetNumberOfTuples(s.poses.size());
    vtkSmartPointer<vtkDoubleArray> der_normals = vtkSmartPointer<vtkDoubleArray>::New();;
    der_normals->SetNumberOfComponents(3); //3d normals (ie x,y,z)
    der_normals->SetNumberOfTuples(s.poses.size());

    int idx = 0;
    for(auto& pose : s.poses)
    {
      Isometry3d pose_eigen;
      Vector3d point, vx, vy, vz;
      tf::poseMsgToEigen(pose, pose_eigen);
      point = pose_eigen.translation();
      vx = pose_eigen.linear().col(0);
      vx *= -1.0;
      vy = pose_eigen.linear().col(1);
      vz = pose_eigen.linear().col(2);
      points->InsertNextPoint(point.data());
      line_normals->SetTuple(idx,vz.data());
      der_normals->SetTuple(idx,vx.data());

      idx++;
    }
    tpp.line->SetPoints(points);
    tpp.line->GetPointData()->SetNormals(line_normals);
    tpp.derivatives->GetPointData()->SetNormals(der_normals);
    tpp.derivatives->SetPoints(points);

    noether_tp.push_back(tpp);
  }
  return noether_tp;
}

std::vector<geometry_msgs::PoseArray> toPosesMsgs(const std::vector<tool_path_planner::ProcessPath>& paths)
{
  std::vector<geometry_msgs::PoseArray> poseArrayVector;
  for(int j = 0; j < paths.size(); ++j)
  {
     geometry_msgs::PoseArray poses;
     poses.header.seq = j;
     poses.header.stamp = ros::Time::now();
     poses.header.frame_id = "0";
     for(int k = 0; k < paths[j].line->GetPoints()->GetNumberOfPoints(); ++k)
     {
        geometry_msgs::Pose pose;
        double pt[3];

        // Get the point location
        paths[j].line->GetPoints()->GetPoint(k, pt);
        pose.position.x = pt[0];
        pose.position.y = pt[1];
        pose.position.z = pt[2];

        // Get the point normal and derivative for creating the 3x3 transform
        double* norm =
            paths[j].line->GetPointData()->GetNormals()->GetTuple(k);
        double* der =
            paths[j].derivatives->GetPointData()->GetNormals()->GetTuple(k);

        // perform cross product to get the third axis direction
        Eigen::Vector3d u(norm[0], norm[1], norm[2]);
        Eigen::Vector3d v(der[0], der[1], der[2]);
        Eigen::Vector3d w = u.cross(v);
        w.normalize();

        // after first cross product, u and w will be orthogonal.
        // Perform cross product one more time to make sure that v is perfectly
        // orthogonal to u and w
        v = u.cross(w);
        v.normalize();

        Eigen::Affine3d epose = Eigen::Affine3d::Identity();
        epose.matrix().col(0).head<3>() = v;
        epose.matrix().col(1).head<3>() = -w;
        epose.matrix().col(2).head<3>() = u;
        epose.matrix().col(3).head<3>() = Eigen::Vector3d(pt[0], pt[1], pt[2]);

        tf::poseEigenToMsg(epose, pose);

        // push back new matrix (pose and orientation), this makes one long
        // vector may need to break this up more
        poses.poses.push_back(pose);

      }
      poseArrayVector.push_back(poses);
    }

  return poseArrayVector;
}

tool_path_planner::ProcessTool fromTppMsg(const noether_msgs::ToolPathConfig& tpp_msg_config)
{
  return tool_path_planner::ProcessTool{.pt_spacing = tpp_msg_config.pt_spacing,
    .line_spacing = tpp_msg_config.line_spacing,
    .tool_offset = tpp_msg_config.tool_offset,
    .intersecting_plane_height = tpp_msg_config.intersecting_plane_height,
    .min_hole_size = tpp_msg_config.min_hole_size,
    .min_segment_size = tpp_msg_config.min_segment_size,
    .raster_angle = tpp_msg_config.raster_angle,
    .raster_wrt_global_axes = static_cast<bool>(tpp_msg_config.raster_wrt_global_axes),
    .generate_extra_rasters = static_cast<bool>(tpp_msg_config.generate_extra_rasters)
  };
}

noether_msgs::ToolPathConfig toTppMsg(const tool_path_planner::ProcessTool& tool_config)
{
  noether_msgs::ToolPathConfig tpp_config_msg;
  tpp_config_msg.pt_spacing = tool_config.pt_spacing;
  tpp_config_msg.line_spacing = tool_config.line_spacing;
  tpp_config_msg.tool_offset = tool_config.tool_offset;
  tpp_config_msg.intersecting_plane_height = tool_config.intersecting_plane_height;
  tpp_config_msg.min_hole_size = tool_config.min_hole_size;
  tpp_config_msg.min_segment_size = tool_config.min_segment_size;
  tpp_config_msg.raster_angle = tool_config.raster_angle;
  tpp_config_msg.raster_wrt_global_axes = tool_config.raster_wrt_global_axes;
  tpp_config_msg.generate_extra_rasters = tool_config.generate_extra_rasters;

  return std::move(tpp_config_msg);
}

Eigen::Matrix3d toRotationMatrix(const Eigen::Vector3d& vx, const Eigen::Vector3d& vy,
                                                    const Eigen::Vector3d& vz)
{
  using namespace Eigen;
  Matrix3d rot;
  rot.block(0,0,1,3) = Vector3d(vx.x(), vy.x(), vz.x()).array().transpose();
  rot.block(1,0,1,3) = Vector3d(vx.y(), vy.y(), vz.y()).array().transpose();
  rot.block(2,0,1,3) = Vector3d(vx.z(), vy.z(), vz.z()).array().transpose();
  return rot;
}


bool createPoseArray(const pcl::PointCloud<pcl::PointNormal>& cloud_normals, const std::vector<int>& indices,
                     geometry_msgs::PoseArray& poses)
{
  using namespace pcl;
  using namespace Eigen;

  geometry_msgs::Pose pose_msg;
  Isometry3d pose;
  Vector3d x_dir, z_dir, y_dir;
  std::vector<int> cloud_indices;
  if(indices.empty())
  {
    cloud_indices.resize(cloud_normals.size());
    std::iota(cloud_indices.begin(), cloud_indices.end(), 0);
  }
  else
  {
    cloud_indices.assign(indices.begin(), indices.end());
  }

  for(std::size_t i = 0; i < cloud_indices.size() - 1; i++)
  {
    std::size_t idx_current = cloud_indices[i];
    std::size_t idx_next = cloud_indices[i+1];
    if(idx_current >= cloud_normals.size() || idx_next >= cloud_normals.size())
    {
      CONSOLE_BRIDGE_logError("Invalid indices (current: %lu, next: %lu) for point cloud were passed",
                              idx_current, idx_next);
      return false;
    }
    const PointNormal& p1 = cloud_normals[idx_current];
    const PointNormal& p2 = cloud_normals[idx_next];
    x_dir = (p2.getVector3fMap() - p1.getVector3fMap()).normalized().cast<double>();
    z_dir = Vector3d(p1.normal_x, p1.normal_y, p1.normal_z).normalized();
    y_dir = z_dir.cross(x_dir).normalized();

    pose = Translation3d(p1.getVector3fMap().cast<double>());
    pose.matrix().block<3,3>(0,0) = tool_path_planner::toRotationMatrix(x_dir, y_dir, z_dir);
    tf::poseEigenToMsg(pose,pose_msg);

    poses.poses.push_back(pose_msg);
  }

  // last pose
  pose_msg = poses.poses.back();
  pose_msg.position.x = cloud_normals[cloud_indices.back()].x;
  pose_msg.position.y = cloud_normals[cloud_indices.back()].y;
  pose_msg.position.z = cloud_normals[cloud_indices.back()].z;
  poses.poses.push_back(pose_msg);

  return true;
}

}

