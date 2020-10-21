/*
 * Copyright (c) 2018, Southwest Research Institute
 * All rights reserved.*
 * tool_path_generator.cpp
 *
 *  Created on: Nov 15, 2018
 *      Author: Jorge Nicho
 */

#include <numeric>

#include <ros/assert.h>
#include <eigen_conversions/eigen_msg.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkGenericCell.h>
#include <vtkCellLocator.h>
#include <vtkDoubleArray.h>
#include <vtkIntersectionPolyDataFilter.h>
#include <vtkParametricFunctionSource.h>
#include <vtkOBBTree.h>
#include <vtkTriangle.h>
#include <vtk_viewer/vtk_utils.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <boost/make_shared.hpp>
#include <console_bridge/console.h>
#include <noether_conversions/noether_conversions.h>
#include <tool_path_planner/surface_walk_raster_generator.h>
#include <tool_path_planner/utilities.h>

static const std::size_t MAX_ATTEMPTS = 1000;
static const double ANGLE_CORRECTION_THRESHOLD = (150.0/180.0)*M_PI;
static const double EXTRUDE_EXTEND_PERCENTAGE = 1.5;
static const double RAY_INTERSECTION_TOLERANCE = 0.001;

namespace tool_path_planner
{

/**
 * @brief Structure to store the first and last positions of the path segments. This is all the info
 * we need to make decisions about path order as we currently don't split paths up.
 *
 * The \e a and \e b fields do not indicate any spatial relationship and are merely to uniquely
 * identify the two end points of a line segment.
 *
 * The \id field here is used to store the index of the input path that corresponds to this
 * segment. These points are sorted so this field is used to reconstruct the result at the end.
 */
struct PathEndPoints
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d a;
  Eigen::Vector3d b;
  size_t id;
};

/**
 * @brief A structure to hold the path segments and their direction. The \e id field indicates the
 * index into the PathEndPoints sequence that corresponds to this method. The \e from_a field is used
 * to indicate whether the path should go A to B or B to A. A true value indicates A to B.
 */
struct SequencePoint
{
  size_t id;
  bool from_a;
};

/**
 * @brief From a sequence of path segments, this method extracts the end points and puts them into
 * a new reference frame. As segments are indivisible, we only need the extremes for sorting them.
 * @param segments The source of path segment data
 * @param ref_rotation A transform from the origin to a reference frame which we want all the end
 * points in. The paths in \e segments are considered to be in the origin frame.
 * @return A sequence of end points in the reference frame of \e ref_rotation
 */
std::vector<PathEndPoints> toEndPoints(const std::vector<EigenSTL::vector_Affine3d>& segments,
                                       const Eigen::Quaterniond& ref_rotation)
{
  // Ref rotation is the Target Frame w.r.t. Origin
  // The points are all w.r.t. Origin, ergo we have to pre-multiply by the inverse of ref_rotation
  // to get the new points in the Target Frame
  Eigen::Affine3d ref_inv;
  ref_inv = ref_rotation.inverse();

  std::vector<PathEndPoints> result;
  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    const auto& s = segments[i];
    Eigen::Vector3d a = (ref_inv * s.front()).translation();
    Eigen::Vector3d b = (ref_inv * s.back()).translation();
    result.push_back({a, b, i});
  }
  return result;
}

/**
 * @brief Reconstructs a set of PoseArray objects using the given set of sequence points which contain
 * indices into the \e end_points array which reference the original \e in trajectory.
 * @param in The original trajectory
 * @param seqs The sequence points whose 'id' field reaches into the \e end_points vector
 * @param end_points The set of end points whose 'id' field reaches into the \e in vector
 * @return A new pose array constructed with the sequence ordering from the \e in trajectory
 */
std::vector<geometry_msgs::PoseArray> makeSequence(const std::vector<geometry_msgs::PoseArray>& in,
                                                   const std::vector<SequencePoint>& seqs,
                                                   const std::vector<PathEndPoints>& end_points)
{
  assert(in.size() == seqs.size());
  std::vector<geometry_msgs::PoseArray> rs;
  rs.reserve(in.size());

  for (const auto& seq : seqs)
  {
    rs.push_back(in[end_points[seq.id].id]); // seq.id points to end_points; end_points.id points to in
    if (!seq.from_a) // The 'in' trajectory has segments that are always A to B
    {
      std::reverse(rs.back().poses.begin(), rs.back().poses.end());
    }
  }

  return rs;
}

/**
 * @brief Computes the 'average' quaternion from an input set of them.
 * See http://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
 * See http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf
 *
 * I don't have a great way of detecting the cases where the result isn't really meaningful,
 * e.g. a set of rotations spread evenly through rotational space.
 */
Eigen::Quaterniond average(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& qs)
{
  Eigen::MatrixXd Q (4, qs.size());

  for (std::size_t i = 0; i < qs.size(); ++i)
  {
    Q.col(static_cast<Eigen::Index>(i)) = qs[i].coeffs();
  }

  Eigen::MatrixXd Q_prime = Q * Q.transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(Q_prime);

  Eigen::VectorXd eigen_vals = eigensolver.eigenvalues();
  Eigen::MatrixXd eigen_vecs = eigensolver.eigenvectors();

  int max_idx = 0;
  double max_value = 0.0;
  for (int i = 0; i < eigen_vals.size(); ++i)
  {
    if (eigen_vals(i) > max_value)
    {
      max_idx = i;
      max_value = eigen_vals(i);
    }
  }

  Eigen::VectorXd coeffs = eigen_vecs.col(max_idx);
  Eigen::Quaterniond avg_quat (coeffs(3), coeffs(0), coeffs(1), coeffs(2));
  return avg_quat;
}

// Helpers to go from pose arrays to Eigen vectors of Poses
EigenSTL::vector_Affine3d toEigen(const geometry_msgs::PoseArray& p)
{
  EigenSTL::vector_Affine3d rs (p.poses.size());
  std::transform(p.poses.begin(), p.poses.end(), rs.begin(), [] (const geometry_msgs::Pose& pose)
  {
    Eigen::Affine3d e;
    tf::poseMsgToEigen(pose, e);
    return e;
  });
  return rs;
}

// Helpers to go from pose arrays to Eigen vectors of Poses
std::vector<EigenSTL::vector_Affine3d> toEigen(const std::vector<geometry_msgs::PoseArray>& ps)
{
  std::vector<EigenSTL::vector_Affine3d> rs (ps.size());
  std::transform(ps.begin(), ps.end(), rs.begin(), [] (const geometry_msgs::PoseArray& poses)
  {
    return toEigen(poses);
  });
  return rs;
}

// Gets the average quaternion rotation of a set of poses
Eigen::Quaterniond averageQuaternion(const EigenSTL::vector_Affine3d& poses)
{
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> qs;
  qs.reserve(poses.size());

  for (const auto& p : poses)
  {
    qs.push_back(Eigen::Quaterniond(p.rotation()));
  }

  return average(qs);
}

/**
 * @brief Returns the index of the path segment with the largest end-point displacement
 * (first.position - last.position) in \e segments
 *
 * We assume that segments is non-empty. Will return 0 in that case.
 */
std::size_t longestSegment(const std::vector<EigenSTL::vector_Affine3d>& segments)
{
  std::size_t max_index = 0;
  double max_dist = 0.0;

  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    auto dist = (segments[i].front().translation() - segments[i].back().translation()).squaredNorm();
    if (dist > max_dist)
    {
      max_index = i;
      max_dist = dist;
    }
  }
  return max_index;
}

/**
 * @brief Given \e input, a set of path segments, this algorithm will produce a new set of segments
 * that is the result of re-ordering the points left to right relative to the nominal 'cut' direction.
 */
std::vector<geometry_msgs::PoseArray> sequence(const std::vector<geometry_msgs::PoseArray>& input)
{
  if (input.empty())
  {
    return {};
  }

  auto eigen_poses = toEigen(input);
  // We need to compute the 'nominal' cut direction of the surface paths
  // We do that by picking the "largest" cut first
  auto longest_segment_idx = longestSegment(eigen_poses);
  // Then we find the average rotation
  Eigen::Quaterniond avg_quaternion = averageQuaternion(eigen_poses[longest_segment_idx]);
  // And get the end points of the path segments in that rotational frame, such that paths
  // run along the X direction and are spaced out ~ in Y
  auto end_points = toEndPoints(eigen_poses, avg_quaternion);

  // Sort end points, -y to y
  std::sort(end_points.begin(), end_points.end(), [] (const PathEndPoints& lhs, const PathEndPoints& rhs)
  {
    auto lhs_value = std::min(lhs.a.y(), lhs.b.y());
    auto rhs_value = std::min(rhs.a.y(), rhs.b.y());
    return lhs_value < rhs_value;
  });

  // A helper function to get the starting point of a transition given a sequence number and
  // whether we started at A or B.
  auto current_position = [&end_points](const SequencePoint& p) {
    if (p.from_a) // If we came from A, we're now at B
      return end_points[p.id].b;
    else // if we came from B, we're not at A
      return end_points[p.id].a;
  };

  std::vector<SequencePoint> sequence;
  sequence.reserve(input.size());

  // We always start at the first end_point, position A
  sequence.push_back({0, true});

  for (std::size_t i = 1; i < end_points.size(); ++i)
  {
    // We need to determine if A or B of the next path is closer to the current position
    const Eigen::Vector3d current_pos = current_position(sequence.back());

    const auto dist_a = (end_points[i].a - current_pos).squaredNorm();
    const auto dist_b = (end_points[i].b - current_pos).squaredNorm();

    const auto from_a = dist_a < dist_b;
    sequence.push_back({i, from_a});
  }

  // Re-order the original inputs and produce a new segment.
  return makeSequence(input, sequence, end_points);
}

/////////////////////////////////////////
/// NEW CODE BELOW
///////////////////////////////////////////////////

//std::vector<noether_msgs::ToolRasterPath> toPosesMsgs(const std::vector<tool_path_planner::ProcessPath>& paths)
//{
//  std::vector<noether_msgs::ToolRasterPath> results;

//  for(std::size_t j = 0; j < paths.size(); ++j)
//  {
//     geometry_msgs::PoseArray poses;
//     poses.header.seq = j;
//     poses.header.stamp = ros::Time::now();
//     poses.header.frame_id = "0";
//     for(int k = 0; k < paths[j].line->GetPoints()->GetNumberOfPoints(); ++k)
//     {
//        geometry_msgs::Pose pose;
//        double pt[3];

//        // Get the point location
//        paths[j].line->GetPoints()->GetPoint(k, pt);
//        pose.position.x = pt[0];
//        pose.position.y = pt[1];
//        pose.position.z = pt[2];

//        // Get the point normal and derivative for creating the 3x3 transform
//        double* norm =
//            paths[j].line->GetPointData()->GetNormals()->GetTuple(k);
//        double* der =
//            paths[j].derivatives->GetPointData()->GetNormals()->GetTuple(k);

//        // perform cross product to get the third axis direction
//        Eigen::Vector3d u(norm[0], norm[1], norm[2]);
//        Eigen::Vector3d v(der[0], der[1], der[2]);
//        Eigen::Vector3d w = u.cross(v);
//        w.normalize();

//        // after first cross product, u and w will be orthogonal.
//        // Perform cross product one more time to make sure that v is perfectly
//        // orthogonal to u and w
//        v = u.cross(w);
//        v.normalize();

//        Eigen::Affine3d epose = Eigen::Affine3d::Identity();
//        epose.matrix().col(0).head<3>() = v;
//        epose.matrix().col(1).head<3>() = -w;
//        epose.matrix().col(2).head<3>() = u;
//        epose.matrix().col(3).head<3>() = Eigen::Vector3d(pt[0], pt[1], pt[2]);

//        tf::poseEigenToMsg(epose, pose);

//        // push back new matrix (pose and orientation), this makes one long
//        // vector may need to break this up more
//        poses.poses.push_back(pose);

//      }
//      noether_msgs::ToolRasterPath tool_path;
//      tool_path.paths.push_back(poses);
//      results.push_back(tool_path);
//    }

//  return results;
//}

void flipPointOrder(ProcessPath &path)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
  points = path.line->GetPoints();

  // flip point order
  for(long i = points->GetNumberOfPoints() - 1; i >= 0; --i)
  {
    points2->InsertNextPoint(points->GetPoint(i));
  }
  path.line->SetPoints(points2);

  // flip normal order
  vtkSmartPointer<vtkDataArray> norms = path.line->GetPointData()->GetNormals();
  vtkSmartPointer<vtkDoubleArray> new_norms = vtkSmartPointer<vtkDoubleArray>::New();
  new_norms->SetNumberOfComponents(3);

  for(long i = norms->GetNumberOfTuples() - 1; i >= 0; --i)
  {
    double* ptr = norms->GetTuple(i);
    new_norms->InsertNextTuple(ptr);
  }
  path.line->GetPointData()->SetNormals(new_norms);

  // flip point order
  points = path.derivatives->GetPoints();
  vtkSmartPointer<vtkPoints> dpoints2 = vtkSmartPointer<vtkPoints>::New();
  for(long i = points->GetNumberOfPoints() - 1; i >= 0; --i)
  {
    dpoints2->InsertNextPoint(points->GetPoint(i));
  }
  path.derivatives->SetPoints(dpoints2);

  // flip derivative directions
  vtkDataArray* ders = path.derivatives->GetPointData()->GetNormals();
  vtkSmartPointer<vtkDoubleArray> new_ders = vtkSmartPointer<vtkDoubleArray>::New();
  new_ders->SetNumberOfComponents(3);
  for(long i = ders->GetNumberOfTuples() -1; i >= 0; --i)
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

std::vector<geometry_msgs::PoseArray> toPosesMsgs(const std::vector<tool_path_planner::ProcessPath>& paths)
{
  std::vector<geometry_msgs::PoseArray> poseArrayVector;
  for(std::size_t j = 0; j < paths.size(); ++j)
  {
     geometry_msgs::PoseArray poses;
     poses.header.seq = j;
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

/**
 * @brief computes the angle between two vectors
 * @param v1
 * @param v2
 * @return The angle in radians
 */
double computeAngle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  return std::acos(v1.dot(v2)/(v1.norm() * v2.norm()));
}

/**
 * @brief computes the squared distance between two 3d vectors
 * @param pt1
 * @param pt2
 * @return
 */
double computeSquaredDistance(const std::vector<double>& pt1, const std::vector<double>& pt2)
{
  if(pt1.size() != 3 || pt2.size() != 3)
  {
    return 0;
  }
  return (pow(pt1[0] - pt2[0], 2.0) + pow(pt1[1] - pt2[1], 2.0 ) + pow((pt1[2] - pt2[2]), 2.0 ));
}

/**
 * @brief findClosestPoint Finds the closest point in a list to a target point
 * @param pt The target point
 * @param pts The list of points to search for the closest point
 * @return The index of the closest point
 */
int findClosestPoint(const std::vector<double>& pt,  const std::vector<std::vector<double> >& pts)
{
  double min = std::numeric_limits<double>::max();
  int index = -1;
  for(std::size_t i = 0; i < pts.size(); ++i)
  {
    double d = computeSquaredDistance(pt, pts[i]);
    if(d < min)
    {
      index = static_cast<int>(i);
      min = d;
    }
  }
  return index;
}

bool SurfaceWalkRasterGenerator::setConfiguration(const Config& config)
{
  config_ = config;
}

void SurfaceWalkRasterGenerator::setInput(pcl::PolygonMesh::ConstPtr mesh)
{
  auto mesh_data = vtkSmartPointer<vtkPolyData>::New();
  pcl::VTKUtils::mesh2vtk(*mesh, mesh_data);
  mesh_data->BuildLinks();
  mesh_data->BuildCells();
  setInput(mesh_data);
}

void SurfaceWalkRasterGenerator::setInput(vtkSmartPointer<vtkPolyData> mesh)
{
  if (!mesh_data_)
    mesh_data_ = vtkSmartPointer<vtkPolyData>::New();

  mesh_data_->DeepCopy(mesh);

  if(mesh_data_->GetPointData()->GetNormals() && mesh_data_->GetCellData()->GetNormals() )
  {
    CONSOLE_BRIDGE_logError("Normal data is not available", getName().c_str());
  }
  else
  {
    CONSOLE_BRIDGE_logWarn("%s generating normal data", getName().c_str());
    vtkSmartPointer<vtkPolyDataNormals> normal_generator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normal_generator->SetInputData(mesh_data_);
    normal_generator->ComputePointNormalsOn();
    normal_generator->SetComputeCellNormals(!mesh_data_->GetCellData()->GetNormals());
    normal_generator->SetFeatureAngle(M_PI_2);
    normal_generator->SetSplitting(true);
    normal_generator->SetConsistency(true);
    normal_generator->SetAutoOrientNormals(false);
    normal_generator->SetFlipNormals(false);
    normal_generator->SetNonManifoldTraversal(false);
    normal_generator->Update();


    if ( !mesh_data_->GetPointData()->GetNormals())
    {
      mesh_data_->GetPointData()->SetNormals(normal_generator->GetOutput()->GetPointData()->GetNormals());
    }

    if ( !mesh_data_->GetCellData()->GetNormals() )
    {
      mesh_data_->GetCellData()->SetNormals(normal_generator->GetOutput()->GetCellData()->GetNormals());
    }
  }

  // build cell locator and kd_tree to recover normals later on
  kd_tree_ = vtkSmartPointer<vtkKdTreePointLocator>::New();
  kd_tree_->SetDataSet(mesh_data_);
  kd_tree_->BuildLocator();

  cell_locator_ = vtkSmartPointer<vtkCellLocator>::New();
  cell_locator_->SetDataSet(mesh_data_);
  cell_locator_->BuildLocator();

  bsp_tree_ = vtkSmartPointer<vtkModifiedBSPTree>::New();
  bsp_tree_->SetDataSet(mesh_data_);
  bsp_tree_->BuildLocator();

  // Add display for debugging
  if(config_.debug)
  {
    debug_viewer_.removeAllDisplays();
    std::vector<float> color(3);
    color[0] = 0.9f; color[1] = 0.9f; color[2] = 0.9f;
    debug_viewer_.addPolyDataDisplay(mesh_data_, color);
  }
}

void SurfaceWalkRasterGenerator::setInput(const shape_msgs::Mesh& mesh)
{
  pcl::PolygonMesh::Ptr pcl_mesh = boost::make_shared<pcl::PolygonMesh>();
  noether_conversions::convertToPCLMesh(mesh,*pcl_mesh);
  setInput(pcl_mesh);
}

vtkSmartPointer<vtkPolyData> SurfaceWalkRasterGenerator::getInput()
{
  return mesh_data_;
}

boost::optional<ToolPaths> SurfaceWalkRasterGenerator::generate()
{
  // Need to call getFirstPath or other method to generate the first path
  // If no paths exist, there is nothing to create offset paths from
  if(paths_.size() != 1)
  {
    ProcessPath first_path;
    if(!getFirstPath(first_path))
    {
      return boost::none;
    }
  }

  // could potentially make these two loops run in parallel but then we would need to add locks on the data
  bool done = false;
  int max = MAX_ATTEMPTS;
  int count = 0;

  // From existing cutting plane, create more offset planes in one direction
  while(!done && count < max)
  {
    ProcessPath path2;
    if(getNextPath(paths_.back(), path2, config_.raster_spacing))
    {
      paths_.push_back(path2);
    }
    else
    {
      done = true;
    }
    ++count;
  }

  // From existing cutting plane, create more offset planes in opposite direction
  count = 0;
  done = false;
  while(!done && count < max)
  {
    ProcessPath path2;
    if(getNextPath(paths_.front(), path2, -config_.raster_spacing))
    {
      paths_.insert(paths_.begin(), path2 );
    }
    else
    {
      done = true;
    }

    ++count;
  }

  // clear all but the first (mesh) display
  if(config_.debug)
  {
    int num_obj = static_cast<int>(debug_viewer_.getNumberOfDisplayObjects()) - 1;
    for(int i = 0; i < num_obj; ++i)
    debug_viewer_.removeObjectDisplay(debug_viewer_.getNumberOfDisplayObjects() - 1);
  }


  std::vector<ProcessPath> new_paths;
  std::vector<int> delete_paths;
  for(std::size_t i = 0; i < paths_.size(); ++i)
  {
    std::vector<ProcessPath> out_paths;
    if(checkPathForHoles(paths_[i], out_paths))
    {
      // mark a path to delete
      delete_paths.push_back(static_cast<int>(i));
      for(std::size_t j = 0; j < out_paths.size(); ++j)
      {
        new_paths.push_back(out_paths[j]); // save all new paths
      }
    }
  }

  // Get the first and last paths before checking for holes. Paths
  // containing holes will be removed, broken up, and inserted at the
  // end. This disrupts the order and makes the last path difficult to
  // find during generation of extra rasters.
  ProcessPath first = paths_.front();
  ProcessPath last = paths_.back();

  // if paths need to be modified, delete all old paths (starting at the back) and insert new ones
  if(!delete_paths.empty())
  {
    CONSOLE_BRIDGE_logInform("Deleting %d paths", delete_paths.size());
  }
  for(int i = static_cast<int>(delete_paths.size()) - 1; i >= 0 ; --i )
  {
    paths_.erase(paths_.begin() + delete_paths[static_cast<std::size_t>(i)]);
  }

  // flip any errant normals that tend to happen at edge of the mesh
  for(std::size_t i = 0; i < new_paths.size(); i++)
  {
    // find the average normal along each new_path
    Eigen::Vector3d sum_of_normal = Eigen::Vector3d::Zero();
    for(int j = 0; j<new_paths[i].line->GetPoints()->GetNumberOfPoints(); j++)
    {
      Eigen::Vector3d current_normal = Eigen::Vector3d::Zero();
      new_paths[i].line->GetPointData()->GetNormals()->GetTuple(j,current_normal.data());
      sum_of_normal += current_normal;
    }
    sum_of_normal.normalize(); // this is now the average normal
    CONSOLE_BRIDGE_logDebug("Line Normal xyz = %f, %f, %f, %f", sum_of_normal.x(), sum_of_normal.y(), sum_of_normal.z());
    vtkDataArray* normals = new_paths[i].line->GetPointData()->GetNormals();

    // flip normal directions if it is in a different direction from the average of normal for the line
    vtkDataArray* line_normals = new_paths[i].line->GetPointData()->GetNormals();
    vtkSmartPointer<vtkDoubleArray> new_line_normals = vtkSmartPointer<vtkDoubleArray>::New();
    new_line_normals->SetNumberOfComponents(3);
    for(long j = line_normals ->GetNumberOfTuples() - 1; j >= 0; --j)
    {
      Eigen::Vector3d current_normal = Eigen::Vector3d::Zero();
      new_paths[i].line->GetPointData()->GetNormals()->GetTuple(j,current_normal.data());
      double dp = current_normal.dot(sum_of_normal);
      CONSOLE_BRIDGE_logDebug("Line %d waypoint %d dot product: %d", i, j, dp);
      if(dp < 0)
      {
        CONSOLE_BRIDGE_logDebug("Flipping line %d waypoint $d", i, j);
        double* pt = line_normals ->GetTuple(static_cast<long>(i));
        pt[0] *= -1;
        pt[1] *= -1;
        pt[2] *= -1;
        new_line_normals->InsertNextTuple(pt);
      }
      else
      {
        double* pt = line_normals ->GetTuple(static_cast<long>(i));
        new_line_normals->InsertNextTuple(pt);
      }
    }
    new_paths[i].line->GetPointData()->SetNormals(new_line_normals);

  }

  // Insert new meshes
  for(std::size_t i = 0; i < new_paths.size(); ++i)
  {
    paths_.push_back(new_paths[i]);

    if(config_.debug)  // cutting mesh display
    {
      std::vector<float> color(3);
      color[0] = 0.8f; color[1] = 0.8f; color[2] = 0.8f;

      debug_viewer_.addPolyDataDisplay(new_paths[i].intersection_plane, color);
      debug_viewer_.renderDisplay();
    }
  }

  // If requested, generate extra paths extending past the edge of the part
  if (config_.generate_extra_rasters == true && paths_.size() >= 1)
  {
    ProcessPath first_path, last_path;
    if (getExtraPath(first, first_path, -config_.raster_spacing))
    {
      paths_.insert(paths_.begin(), first_path );
    }
    else
    {
      CONSOLE_BRIDGE_logError("Failed to generate path off leading edge");
    }
    if (getExtraPath(last, last_path, config_.raster_spacing))
    {
      paths_.push_back(last_path);
    }
    else
    {
      CONSOLE_BRIDGE_logError("Failed to generate path off trailing edge");
    }
  }

  if(paths_.empty())
  {
    ROS_ERROR("All tool paths generated are empty");
    return boost::none;
  }

  ROS_INFO("Found tool paths for input mesh!");

  // converting to pose array
  std::vector<geometry_msgs::PoseArray> tool_path_poses = toPosesMsgs(paths_);

  // rearranging
  tool_path_poses = sequence(tool_path_poses);

  // Convert to results struction
  ToolPaths results;
  for (const auto& rp : tool_path_poses)
  {
    ToolPathSegment tps;
    for (const auto& p : rp.poses)
    {
      Eigen::Isometry3d epose;
      tf::poseMsgToEigen(p, epose);
      tps.push_back(epose);
    }
    ToolPath tp;
    tp.push_back(tps);
    results.push_back(tp);
  }

  return results;
}

std::string SurfaceWalkRasterGenerator::getName() const
{
  return getClassName<decltype(*this)>();
}

bool SurfaceWalkRasterGenerator::getFirstPath(ProcessPath& path)
{
  // clear old paths before creating new
  paths_.clear();

  // generate first path according to algorithm in createStartCurve()
  vtkSmartPointer<vtkPolyData> start_curve = vtkSmartPointer<vtkPolyData>::New();
  start_curve = createStartCurve();

  // get input mesh bounds
  double bounds[6];
  mesh_data_->GetBounds(bounds);
  double x = fabs(bounds[1] - bounds[0]);
  double y = fabs(bounds[3] - bounds[2]);
  double z = fabs(bounds[5] - bounds[4]);

  // Use the bounds to determine how large to make the cutting mesh
  double max = x > y ? x : y;
  max = max > z ? max : z;

  vtkSmartPointer<vtkPolyData> cutting_mesh = extrudeSplineToSurface(start_curve, max);
  if(!cutting_mesh)
  {
    return false;
  }

  if(config_.debug)  // cutting mesh display
  {
    std::vector<float> color(3);
    color[0] = 0.8f; color[1] = 0.8f; color[2] = 0.8f;

    debug_viewer_.addPolyDataDisplay(cutting_mesh, color);
    debug_viewer_.renderDisplay();
    debug_viewer_.removeObjectDisplay(debug_viewer_.getNumberOfDisplayObjects() - 1);
  }

  // use cutting mesh to find intersection line
  vtkSmartPointer<vtkPolyData> intersection_line = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();

  if(!findIntersectionLine(cutting_mesh, intersection_line, spline))
  {
    CONSOLE_BRIDGE_logDebug("No intersection found");
    return false;
  }

  ProcessPath this_path;
  if(!computeSurfaceLineNormals(intersection_line))
  {
    return false;
  }
  this_path.intersection_plane = intersection_line;

  if(getNextPath(this_path, path, 0.0))
  {
    paths_.push_back(path);
    return true;
  }
  return false;
}

bool SurfaceWalkRasterGenerator::getNextPath(const ProcessPath this_path, ProcessPath& next_path, double dist, bool test_self_intersection)
{
  if(dist == 0.0 && this_path.intersection_plane->GetPoints()->GetNumberOfPoints() < 2)
  {
    CONSOLE_BRIDGE_logDebug("No path offset and no intersection plane given. Cannot generate next path");
    return false;
  }

  vtkSmartPointer<vtkPolyData> offset_line = vtkSmartPointer<vtkPolyData>::New();
  if(dist != 0.0)  // if offset distance given, create an offset surface
  {
    // create offset points in the adjacent line at a distance "dist"
    offset_line = createOffsetLine(this_path.line, this_path.derivatives, dist);
    if(!offset_line)
    {
      return false;
    }

    // create cutting surface  TODO: offset may need to be based upon point bounds
    next_path.intersection_plane = extrudeSplineToSurface(offset_line, config_.intersection_plane_height);
    if(!next_path.intersection_plane)
    {
      return false;
    }
  }
  else  // if no offset distance given, use points in this_path.intersection_plane to create the surface
  {
    // resample points to make sure there the intersection filter works properly
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points = this_path.intersection_plane->GetPoints();
    resamplePoints(points);
    offset_line->SetPoints(points);
    if(!computeSurfaceLineNormals(offset_line))
    {
      return false;
    }

    next_path.intersection_plane = extrudeSplineToSurface(offset_line, config_.intersection_plane_height);
    if(!next_path.intersection_plane)
    {
      return false;
    }
  }

  if(config_.debug)  // offset points and cutting mesh display
  {
    std::vector<float> color(3);
    color[0] = 0.9f; color[1] = 0.2f; color[2] = 0.2f;
    debug_viewer_.addPolyNormalsDisplay(offset_line, color, config_.point_spacing);

    color[0] = 0.8f; color[1] = 0.8f; color[2] = 0.8f;
    debug_viewer_.addPolyDataDisplay(next_path.intersection_plane, color);
    debug_viewer_.renderDisplay();
    debug_viewer_.removeObjectDisplay(debug_viewer_.getNumberOfDisplayObjects() - 2);
  }

  // use surface to find intersection line
  vtkSmartPointer<vtkPolyData> intersection_line = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();

  if(!findIntersectionLine(next_path.intersection_plane, intersection_line, spline))
  {
    CONSOLE_BRIDGE_logDebug("No intersection found for creating spline");
    return false;
  }

  // Check for self intersection (intersection of next path with the last path computed
  // If self-intersection occurs, return false (done planning paths)
  if(test_self_intersection && paths_.size() >= 1)
  {
    vtkSmartPointer<vtkIntersectionPolyDataFilter> intersection_filter =
      vtkSmartPointer<vtkIntersectionPolyDataFilter>::New();
    intersection_filter->SetSplitFirstOutput(0);
    intersection_filter->SetSplitSecondOutput(0);
    intersection_filter->SetInputData( 0, next_path.intersection_plane);
    intersection_filter->SetInputData( 1, paths_.back().intersection_plane);
    intersection_filter->Update();
    if(intersection_filter->GetOutput()->GetPoints()->GetNumberOfPoints() > 0)
    {
      CONSOLE_BRIDGE_logDebug("Self intersection found with back path");
      return false;
    }

    intersection_filter->SetInputData( 1, paths_.front().intersection_plane);
    intersection_filter->Update();
    if(intersection_filter->GetOutput()->GetPoints()->GetNumberOfPoints() > 0)
    {
      CONSOLE_BRIDGE_logDebug("Self intersection found with front path");
      return false;
    }
  }


  if(config_.debug)  // spline display
  {
    std::vector<float> color(3);
    color[0] = 0.2f; color[1] = 0.9f; color[2] = 0.9f;
    vtkSmartPointer<vtkParametricFunctionSource> source = vtkSmartPointer<vtkParametricFunctionSource>::New();
    source->SetParametricFunction(spline);
    source->Update();

    debug_viewer_.addPolyDataDisplay(source->GetOutput(), color);
    debug_viewer_.renderDisplay();
    debug_viewer_.removeObjectDisplay(debug_viewer_.getNumberOfDisplayObjects() - 1);
  }

  //use spline to create interpolated data with normals and derivatives
  vtkSmartPointer<vtkPolyData> points = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPolyData> derivatives = vtkSmartPointer<vtkPolyData>::New();
  smoothData(spline, points, derivatives);

  if(config_.debug)  // points display
  {
    std::vector<float> color(3);
    color[0] = 0.2f; color[1] = 0.2f; color[2] = 0.9f;
    debug_viewer_.addPolyNormalsDisplay(points, color, config_.point_spacing);
    color[0] = 0.9f; color[1] = 0.9f; color[2] = 0.2f;
    debug_viewer_.addPolyNormalsDisplay(derivatives, color, config_.point_spacing);
    debug_viewer_.renderDisplay();
    debug_viewer_.removeObjectDisplay(debug_viewer_.getNumberOfDisplayObjects() - 1);
  }

  if(points->GetPoints()->GetNumberOfPoints() < 2)
  {
    CONSOLE_BRIDGE_logDebug("Number of points after smoothing is less than 2, skip computing path data");
    return false;
  }
  next_path.line = points;
  next_path.spline = spline;
  next_path.derivatives = derivatives;

  // compare start/end points of new line to old line, flip order if necessary
  if(dist != 0.0)  // only flip if offset is non-zero (new_path)
  {
    long length = next_path.line->GetPoints()->GetNumberOfPoints();
    if(vtk_viewer::pt_dist(this_path.line->GetPoints()->GetPoint(0), next_path.line->GetPoints()->GetPoint(0))
       > vtk_viewer::pt_dist(this_path.line->GetPoints()->GetPoint(0), next_path.line->GetPoints()->GetPoint(length-1)))
    {
      flipPointOrder(next_path);
    }
  }

  if(config_.debug)  // points display
  {
    std::vector<float> color(3);
    color[0] = 0.9f; color[1] = 0.9f; color[2] = 0.2f;
    debug_viewer_.addPolyNormalsDisplay(derivatives, color, config_.point_spacing);
    debug_viewer_.renderDisplay();
  }

  return true;
}

bool SurfaceWalkRasterGenerator::getExtraPath(const ProcessPath& last_path, ProcessPath& extra_path, double dist)
{
  if (std::fabs(dist) == 0.0)
  {
    CONSOLE_BRIDGE_logError("No offset given. Cannot generate extra path");
    return false;
  }

  // create offset points in the adjacent line at a distance "dist"
  extra_path.line = vtkSmartPointer<vtkPolyData>::New();
  extra_path.line = createOffsetLine(last_path.line, last_path.derivatives, dist);
  if(!extra_path.line)
  {
    return false;
  }

  // Use the same derivatives (we want to translate in space, not rotate)
  extra_path.derivatives = vtkSmartPointer<vtkPolyData>::New();
  extra_path.derivatives->SetPoints(extra_path.line->GetPoints());
  extra_path.derivatives->GetPointData()->SetNormals(last_path.derivatives->GetPointData()->GetNormals());

  // Don't assign an intersection plane.
  extra_path.intersection_plane = vtkSmartPointer<vtkPolyData>::New();

  // Make sure the spline is using the new points
  extra_path.spline = vtkSmartPointer<vtkParametricSpline>::New();
  extra_path.spline->SetPoints(extra_path.derivatives->GetPoints());
  vtkSmartPointer<vtkPolyData> points = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPolyData> derivatives = vtkSmartPointer<vtkPolyData>::New();
  smoothData(extra_path.spline, points, derivatives);

  // Use the points, normals, and derivatives calculated from the spline
  extra_path.line = points;
  extra_path.derivatives = derivatives;

  return true;
}

bool SurfaceWalkRasterGenerator::checkPathForHoles(const ProcessPath path, std::vector<ProcessPath>& out_paths)
{
  // use cutting mesh to find intersection line
  vtkSmartPointer<vtkPolyData> intersection_line = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();

  if(!findIntersectionLine(path.intersection_plane, intersection_line, spline))
  {
    return false;
  }

  int prev_start_point = 0;

  // For each point on the intersection line, check to see if the points share a common triangle/cell.
  // If they share a cell, there is no hole, if they don't share a cell, then there is a hole and the
  // distance between the two points should be checked to see how large the hole is
  for(int i = 1; i < intersection_line->GetPoints()->GetNumberOfPoints() - 1; ++i)
  {
    // get two adjacent points
    double pt1[3], pt2[3], pcoords[3];
    double weights[3] = {0,0,0};
    intersection_line->GetPoints()->GetPoint(i-1, pt1);
    intersection_line->GetPoints()->GetPoint(i, pt2);

    // calculate the distance between each point
    double diff[3] = {pt2[0] - pt1[0], pt2[1] - pt1[1], pt2[2] - pt1[2]};

    // move each point by a small amount towards each other
    diff[0] *= 0.1;
    diff[1] *= 0.1;
    diff[2] *= 0.1;

    pt1[0] += diff[0];
    pt1[1] += diff[1];
    pt1[2] += diff[2];

    pt2[0] -= diff[0];
    pt2[1] -= diff[1];
    pt2[2] -= diff[2];

    vtkSmartPointer<vtkGenericCell> cell = vtkSmartPointer<vtkGenericCell>::New();
    vtkIdType cell1, cell2;

    bool continous = false;
    double tol = sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]);

    // find the cell that each point lies on
    cell1 = cell_locator_->FindCell(pt1, tol, cell, pcoords, &weights[0]);
    if(cell1 != -1)
    {
      cell2 = cell_locator_->FindCell(pt2, tol, cell, pcoords, &weights[0]);
      if(cell2 != -1)
      {
        // if the cell for point 1 == the cell for point 2, then there is no hole
        if(cell1 == cell2)
        {
          continous = true;
        }
      }
    }

    // If a hole is found, the current path needs to be broken up, create new path from previous start point
    // to current point (i) in the intersection path
    if(!continous)
    {
      // if no shared cell found, check distance and whether or not the path needs to be split
      intersection_line->GetPoints()->GetPoint(i-1, pt1);
      intersection_line->GetPoints()->GetPoint(i, pt2);
      double dist = sqrt(vtk_viewer::pt_dist(&pt1[0], &pt2[0]));

      // split paths if hole is too large
      if(dist > config_.min_hole_size)
      {
        vtkSmartPointer<vtkIdList> ids = vtkSmartPointer<vtkIdList>::New();
        // create new path from prev_start_point to current i
        for(int j = prev_start_point; j < i; ++j)
        {
          ids->InsertNextId(j);
        }

        vtkSmartPointer<vtkPolyData> new_line = vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkPoints> new_points = vtkSmartPointer<vtkPoints>::New();
        intersection_line->GetPoints()->GetPoints(ids, new_points);

        if(new_points->GetNumberOfPoints() >= 2)
        {
          resamplePoints(new_points);
        }
        else
        {
          CONSOLE_BRIDGE_logDebug("Hole segment has less than 2 points, skipping resampling");
        }

        new_line->SetPoints(new_points);

        ProcessPath this_path, new_path;
        computeSurfaceLineNormals(new_line);
        this_path.intersection_plane = new_line;

        if(getNextPath(this_path, new_path, 0.0, false))
        {
          out_paths.push_back(new_path);
        }
        prev_start_point = i;
      }
    }
  }

  // once done looping, make last path (if a break occured)
  if(prev_start_point > 0)
  {
    vtkSmartPointer<vtkIdList> ids = vtkSmartPointer<vtkIdList>::New();
    for(int j = prev_start_point; j < intersection_line->GetPoints()->GetNumberOfPoints(); ++j)
    {
      ids->InsertNextId(j);
    }
    vtkSmartPointer<vtkPolyData> new_line = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> new_points = vtkSmartPointer<vtkPoints>::New();
    intersection_line->GetPoints()->GetPoints(ids, new_points);
    resamplePoints(new_points);
    new_line->SetPoints(new_points);

    ProcessPath this_path, new_path;
    computeSurfaceLineNormals(new_line);
    this_path.intersection_plane = new_line;

    if(getNextPath(this_path, new_path, 0.0, false))
    {
      out_paths.push_back(new_path);
    }
  }

  return out_paths.size() >= 1;

}

vtkSmartPointer<vtkPolyData> SurfaceWalkRasterGenerator::createStartCurve()
{
  // Find weighted center point and normal average of the input mesh
  vtkSmartPointer<vtkCellArray> cell_ids = mesh_data_->GetPolys();
  cell_ids->InitTraversal();

  double avg_center[3] = {0,0,0};
  double avg_norm[3] = {0,0,0};
  double avg_area = 0;
  long num_cells = cell_ids->GetNumberOfCells();

  // iterate through all cells to find averages
  for(int i = 0; i < num_cells; ++i)
  {
    double center[3] = {0,0,0};
    double norm[3] = {0,0,0};
    double area = 0;
    if(getCellCentroidData(i, &center[0], &norm[0], area))
    {
      avg_center[0] += center[0]*area;
      avg_center[1] += center[1]*area;
      avg_center[2] += center[2]*area;
      avg_norm[0] += norm[0]*area;
      avg_norm[1] += norm[1]*area;
      avg_norm[2] += norm[2]*area;
      avg_area += area;
    }
  }

  // calculate the averages; since we are calculating a weighted sum (based on area),
  // divide by the total area in order to get the weighted average
  avg_center[0] /= avg_area;
  avg_center[1] /= avg_area;
  avg_center[2] /= avg_area;
  avg_norm[0] /= avg_area;
  avg_norm[1] /= avg_area;
  avg_norm[2] /= avg_area;

  // Compute Object Oriented Bounding Box and use the max vector for aligning starting curve
  vtkSmartPointer<vtkPolyData> line = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
  line->SetPoints(pts);
  double corner[3];
  double max[3];
  double mid[3];
  double min[3];
  double size[3];

  if(config_.cut_direction[0] || config_.cut_direction[1] || config_.cut_direction[2])
  {
    max[0] = config_.cut_direction[0]; max[1] = config_.cut_direction[1]; max[2] = config_.cut_direction[2];
    avg_center[0] = config_.cut_centroid[0]; avg_center[1] = config_.cut_centroid[1]; avg_center[2] = config_.cut_centroid[2];
  }
  else
  {
    vtkSmartPointer<vtkOBBTree> obb_tree = vtkSmartPointer<vtkOBBTree>::New();
    obb_tree->SetTolerance(0.001);
    obb_tree->SetLazyEvaluation(0);
    obb_tree->ComputeOBB(mesh_data_->GetPoints(), corner, max, mid, min, size);

    // size gives the length of each vector (max, mid, min) in order, normalize the size vector by the max size for comparison
    double m = size[0];
    size[0] /= m;
    size[1] /= m;
    size[2] /= m;

    // ComputeOBB uses PCA to find the principle axes, thus for square objects it returns the diagonals instead
    // of the minimum bounding box.  Compare the first and second axes to see if they are within 1% of each other
    if(size[0] - size[1] < 0.01)
    {
      // if object is square, need to average max and mid in order to get the correct axes of the object
      m = sqrt(max[0] * max[0] + max[1] * max[1] + max[2] * max[2]);
      double temp_max = m;
      max[0] /= m;
      max[1] /= m;
      max[2] /= m;
      m = sqrt(mid[0] * mid[0] + mid[1] * mid[1] + mid[2] * mid[2]);
      mid[0] /= m;
      mid[1] /= m;
      mid[2] /= m;

      max[0] = (max[0] + mid[0]) * temp_max;
      max[1] = (max[1] + mid[1]) * temp_max;
      max[2] = (max[2] + mid[2]) * temp_max;
    }
  }
  // Create additional points for the starting curve
  double pt[3];
  double raster_axis[3];
  if (!config_.raster_wrt_global_axes)
  {
    // Principal axis is longest axis of the bounding box
    Eigen::Vector3d principal_axis(max);
    Eigen::Vector3d rotation_axis(min);
    rotation_axis.normalize();
    // Form rotation by specified angle about smallest axis of the bounding box
    Eigen::Quaterniond rot(Eigen::AngleAxisd(config_.raster_rot_offset, Eigen::Vector3d(rotation_axis)));
    rot.normalize();
    // Rotate principal axis by quaternion to get raster axis
    Eigen::Vector3d raster_axis_eigen = rot.toRotationMatrix() * principal_axis;
    // Copy out the resuts
    raster_axis[0] = raster_axis_eigen.x();
    raster_axis[1] = raster_axis_eigen.y();
    raster_axis[2] = raster_axis_eigen.z();
  }
  else
  {
    // TODO: Add the ability to define a rotation in mesh coordinates that is then projected onto the mesh plane
  }

  // Use raster_axis to create additional points for the starting curve
  pt[0] = avg_center[0] + raster_axis[0];
  pt[1] = avg_center[1] + raster_axis[1];
  pt[2] = avg_center[2] + raster_axis[2];
  line->GetPoints()->InsertNextPoint(pt);

  line->GetPoints()->InsertNextPoint(avg_center);

  pt[0] = avg_center[0] - raster_axis[0];
  pt[1] = avg_center[1] - raster_axis[1];
  pt[2] = avg_center[2] - raster_axis[2];
  line->GetPoints()->InsertNextPoint(pt);

  // set the normal for all points inserted
  vtkSmartPointer<vtkDoubleArray> norms = vtkSmartPointer<vtkDoubleArray>::New();
  norms->SetNumberOfComponents(3);
  for(int i = 0; i < line->GetPoints()->GetNumberOfPoints(); ++i)
  {
    Eigen::Vector3d m(avg_norm[0], avg_norm[1], avg_norm[2]);
    m.normalize();

    double n[3] ={m[0], m[1], m[2]};

    norms->InsertNextTuple(n);
  }
  line->GetPointData()->SetNormals(norms);

  return line;
}

bool SurfaceWalkRasterGenerator::getCellCentroidData(int id, double* center, double* norm, double& area)
{

    vtkCell* cell = mesh_data_->GetCell(id);
    if(cell)
    {
      vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
      double p0[3];
      double p1[3];
      double p2[3];

      triangle->GetPoints()->GetPoint(0, p0);
      triangle->GetPoints()->GetPoint(1, p1);
      triangle->GetPoints()->GetPoint(2, p2);
      triangle->TriangleCenter(p0, p1, p2, center);
      area = vtkTriangle::TriangleArea(p0, p1, p2);

      double* n = mesh_data_->GetCellData()->GetNormals()->GetTuple(id);
      if(n)
      {
        norm[0] = n[0];
        norm[1] = n[1];
        norm[2] = n[2];
      }

      return true;
    }
    else
    {
      return false;
    }

}

bool SurfaceWalkRasterGenerator::findIntersectionLine(vtkSmartPointer<vtkPolyData> cut_surface,
                                       vtkSmartPointer<vtkPolyData>& points,
                                       vtkSmartPointer<vtkParametricSpline>& spline)
{
  // Check to make sure that the input cut surface contains a valid mesh
  if(cut_surface->GetNumberOfCells() < 1)
  {
    CONSOLE_BRIDGE_logError("Number of input cells for calculating intersection is less than 1, cannot compute intersection");
    return false;
  }

  // Find the intersection between the input mesh and given cutting surface
  vtkSmartPointer<vtkIntersectionPolyDataFilter> intersection_filter =
    vtkSmartPointer<vtkIntersectionPolyDataFilter>::New();
  intersection_filter->SetSplitFirstOutput(0);
  intersection_filter->SetSplitSecondOutput(0);
  intersection_filter->SetInputData( 0, mesh_data_);
  intersection_filter->SetInputData( 1, cut_surface );
  intersection_filter->GlobalWarningDisplayOff();

  intersection_filter->Update();

  // Check output of intersection to see if it is valid
  vtkSmartPointer<vtkPolyData> output = intersection_filter->GetOutput();
  if(!output)
  {
    return false;
  }

  // if no intersection found, return false
  vtkSmartPointer<vtkPoints> pts = intersection_filter->GetOutput()->GetPoints();
  if(intersection_filter->GetStatus() == 0 || !pts || pts->GetNumberOfPoints() <= 1)
  {
    return false;
  }

  vtkSmartPointer<vtkPoints> temp_pts = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();

  // get the intersection filter output and find a continous line segment
  poly_data = intersection_filter->GetOutput();
  getConnectedIntersectionLine(poly_data, temp_pts);

  if(temp_pts->GetNumberOfPoints() == 0)
  {
    CONSOLE_BRIDGE_logError("No connected lines were found");
    return false;
  }

  // return points and spline
  points->SetPoints(temp_pts);
  spline->SetPoints(temp_pts);

  return true;

}

void SurfaceWalkRasterGenerator::getConnectedIntersectionLine(vtkSmartPointer<vtkPolyData> line,
                                                         vtkSmartPointer<vtkPoints>& points)
{
  int start = 0;
  std::vector<int> used_ids;
  std::vector<vtkSmartPointer<vtkPoints> > lines;
  long num_points = line->GetNumberOfPoints();

  while(used_ids.size() < static_cast<std::size_t>(num_points))
  {
    for(int i = 0; i < num_points; ++i)
    {
      // if id 'i' is not found, use it as the next index to start with for finding connected lines
      if(std::find(used_ids.begin(), used_ids.end(), i) == used_ids.end())
      {
        start = i;
        break;
      }
    }
    vtkSmartPointer<vtkPoints> temp_points = vtkSmartPointer<vtkPoints>::New();

    // only add the line segment if it is large enough
    double dist = getConnectedIntersectionLine(line, temp_points, used_ids, start);
    if( dist > config_.min_segment_size)
    {
      lines.push_back(temp_points);
    }
  }

  // Now that we have 1 or more lines, check to see if we need to merge/delete lines
  if(lines.size() == 1)
  {
    points = lines[0];  // if only one line, return it
  }
  else if(lines.size() > 1)
  {
    // check ends of each line for merging/deleting
    vtkSmartPointer<vtkPoints> temp_points = vtkSmartPointer<vtkPoints>::New();
    // find largest line to start with
    long size = 0;
    std::size_t index = 0;
    for(std::size_t i = 0; i < lines.size(); ++i)
    {
      if(lines[i]->GetNumberOfPoints() > size)
      {
        index =i;
        size = lines[i]->GetNumberOfPoints();
      }
    }
    temp_points = lines[index];
    lines.erase(lines.begin() + static_cast<long>(index));

    while(lines.size() > 0)
    {
      std::size_t next_index = 0;
      int order = 0;
      double min_dist = std::numeric_limits<double>::max();

      // find the next closest line
      for(std::size_t i = 0; i < lines.size(); ++i)
      {
        // get distance
        double dist1 = vtk_viewer::pt_dist(temp_points->GetPoint(0), lines[i]->GetPoint(0));
        double dist2 = vtk_viewer::pt_dist(temp_points->GetPoint( temp_points->GetNumberOfPoints() - 1 ),
                                           lines[i]->GetPoint(0) );
        double dist3 = vtk_viewer::pt_dist(temp_points->GetPoint(0),
                                           lines[i]->GetPoint(lines[i]->GetNumberOfPoints() - 1) );
        double dist4 = vtk_viewer::pt_dist(temp_points->GetPoint( temp_points->GetNumberOfPoints() - 1 ),
                                           lines[i]->GetPoint(lines[i]->GetNumberOfPoints() - 1) );


        double dist = std::min(dist1, dist2);
        dist = std::min(dist, dist3);
        dist = std::min(dist, dist4);

        // store distance and index if it is the smallest
        // also store the order, used for later concatenation (front->front, back->front, front->back, back->back)
        if(dist < min_dist)
        {
          next_index = i;
          min_dist = dist;

          // determining joining strategy, lg (final continous line "temp_points") and lc (current line
          order = (dist1 == min_dist) ? 1 : order;  // join lines starting points lc(begin) -> lg(begin)
          order = (dist2 == min_dist) ? 2 : order;  // join lg(end) -> lc(begin)
          order = (dist3 == min_dist) ? 3 : order;  // join lg(begin) -> lc(end)
          order = (dist4 == min_dist) ? 4 : order;  // join lg(end) -> lc(end)
        }
      }

      // Use new index to append lines together or delete lines
      // VTK does not support inserting new points at the front so sometimes we need to create a new object to insert points to
      switch (order)
      {
      case 1:
      {
        vtkSmartPointer<vtkPoints> tmp = vtkSmartPointer<vtkPoints>::New();

        for(long i = lines[next_index]->GetNumberOfPoints() - 1 ; i >=0 ; --i)
        {
          double pt[3];
          lines[next_index]->GetPoint(i, pt);
          tmp->InsertNextPoint(pt);
        }
        for(int i = 0; i < temp_points->GetNumberOfPoints(); ++i)
        {
          double pt[3];
          temp_points->GetPoint(i, pt);
          tmp->InsertNextPoint(pt);
        }
        temp_points->SetNumberOfPoints(tmp->GetNumberOfPoints());
        temp_points->DeepCopy(tmp);
        break;
      }
      case 2:
      {
        for(int i = 0; i < lines[next_index]->GetNumberOfPoints(); ++i)
        {
          temp_points->InsertNextPoint(lines[next_index]->GetPoint(i));
        }
        break;
      }
      case 3:
      {
        vtkSmartPointer<vtkPoints> tmp = vtkSmartPointer<vtkPoints>::New();

        for(int i = 0 ; i < lines[next_index]->GetNumberOfPoints(); ++i)
        {
          double pt[3];
          lines[next_index]->GetPoint(i, pt);
          tmp->InsertNextPoint(pt);
        }
        for(int i = 0; i < temp_points->GetNumberOfPoints(); ++i)
        {
          double pt[3];
          temp_points->GetPoint(i, pt);
          tmp->InsertNextPoint(pt);
        }
        temp_points->SetNumberOfPoints(tmp->GetNumberOfPoints());
        temp_points->DeepCopy(tmp);
        break;
      }
      case 4:
      {
        for(long i = lines[next_index]->GetNumberOfPoints() - 1 ; i >= 0; --i)
        {
          temp_points->InsertNextPoint(lines[next_index]->GetPoint(i));
        }
        break;
      }
      default:
        break;
      }
      lines.erase(lines.begin() + static_cast<long>(next_index));

    }// end of while loop
    points = temp_points;
  }// end concatenating lines together

}

double SurfaceWalkRasterGenerator::getConnectedIntersectionLine(vtkSmartPointer<vtkPolyData> line, vtkSmartPointer<vtkPoints>& points, std::vector<int>& used_ids, int start_pt)
{
  int search_location = 0;
  long num_points = line->GetNumberOfPoints();
  long num_lines = line->GetNumberOfLines();

  vtkSmartPointer<vtkCellArray> line_data = vtkSmartPointer<vtkCellArray>::New();
  line_data = line->GetLines();
  line_data->InitTraversal();

  std::vector<long> ids;

  // get line segment for given point
  ids.push_back(start_pt);
  long next_id = start_pt;
  double line_length = 0.0;

  while(static_cast<long>(ids.size()) < num_points)
  {
    // get the next point id, loop through all line to find
    int j = 0;

    // if next_id is already used, exit
    if(std::find(used_ids.begin(), used_ids.end(), next_id) != used_ids.end())
    {
      break;
    }

    line_data->InitTraversal();
    for(j = 0; j < num_lines; ++j)
    {
      vtkSmartPointer<vtkIdList> temp_cell = vtkSmartPointer<vtkIdList>::New();
      line_data->GetNextCell(temp_cell);
      if(temp_cell->GetNumberOfIds() == 0)
      {
        continue;
      }
      if(next_id == temp_cell->GetId(search_location))
      {
        // based on search direction, determine which value to insert in the id list (first or second)
        int location = (search_location + 1) % 2;
        double pt[3], pt_temp[3];
        line->GetPoint(next_id, pt);

        next_id = temp_cell->GetId(location);
        line->GetPoint(next_id, pt_temp);

        // get distance of current line segment and add to total distance
        line_length += sqrt(vtk_viewer::pt_dist(&pt[0], &pt_temp[0]));

        // insert next id
        if(search_location)
        {
          ids.insert(ids.begin(), next_id);
        }
        else
        {
          ids.push_back(next_id);
        }
        break;
      }
    }

    // If we have gone around a complete loop, next_id will equal first id, then break
    if( (next_id == ids.front() && search_location == 0) || (next_id == ids.back() && search_location == 1))
    {
      break;
    }

    // completed loop without finding a match
    if(j == num_lines && static_cast<long>(ids.size()) < num_points && search_location == 0)
    {
        search_location = 1;
        next_id = ids[0];
      // search from the front to connect points
    }
    else if(j == num_lines && search_location == 1)
    {
      break;
    }
  } // end loop getting connected lines

  // copy the ids used into the in_id list
  used_ids.insert(used_ids.end(), ids.begin(), ids.end());

  // Copy the line ids into a VTK list
  vtkSmartPointer<vtkIdList> connected_pts = vtkSmartPointer<vtkIdList>::New();
  connected_pts->SetNumberOfIds(static_cast<long>(ids.size()));
  for(std::size_t i = 0; i < ids.size(); ++i)
  {
    connected_pts->SetId(static_cast<long>(i), ids[i]);
  }

  // set point size and copy points to return
  points->Reset();
  points->Squeeze();
  points->SetNumberOfPoints(static_cast<long>(ids.size()));
  line->GetPoints()->GetPoints(connected_pts, points);

  return line_length;
}

void SurfaceWalkRasterGenerator::resamplePoints(vtkSmartPointer<vtkPoints>& points)
{
  vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
  spline->SetPoints(points);

  long num_line_pts = points->GetNumberOfPoints() / 2.0;

  double u[3], pt[3], d[9]; // u - search point, pt - resulting point, d - unused but still required
  u[0] = u[1] = u[2] = 0;

  vtkSmartPointer<vtkPoints> new_points = vtkSmartPointer<vtkPoints>::New();
  new_points->SetNumberOfPoints(num_line_pts);

  for(int i = 0; i < num_line_pts; ++i)
  {
    u[0] = double(i)/double(num_line_pts - 1);

    // spline->Evaluate() takes a number in the range [0,1]
    spline->Evaluate(u, pt, d);
    new_points->SetPoint(i, pt);
  }

  // calculation of intersection sometimes fails if edges intersect.  Need to move edge points out some to fix.
  double new_pt[3];
  double* pt1;

  // Extend end point
  pt1 = new_points->GetPoint(new_points->GetNumberOfPoints()- 1);
  new_pt[0] = pt1[0];
  new_pt[1] = pt1[1];
  new_pt[2] = pt1[2];

  pt1 = new_points->GetPoint(new_points->GetNumberOfPoints()- 2);
  new_pt[0] +=  (new_pt[0] - pt1[0]);
  new_pt[1] +=  (new_pt[1] - pt1[1]);
  new_pt[2] +=  (new_pt[2] - pt1[2]);
  new_points->SetPoint(new_points->GetNumberOfPoints()-1, new_pt);

  // Extend front point
  pt1 = new_points->GetPoint(0);
  new_pt[0] = pt1[0];
  new_pt[1] = pt1[1];
  new_pt[2] = pt1[2];
  pt1 = new_points->GetPoint(1);
  new_pt[0] +=  (new_pt[0] - pt1[0]);
  new_pt[1] +=  (new_pt[1] - pt1[1]);
  new_pt[2] +=  (new_pt[2] - pt1[2]);
  new_points->SetPoint(0, new_pt);


  points->SetNumberOfPoints(new_points->GetNumberOfPoints());
  points->DeepCopy(new_points);
}

void SurfaceWalkRasterGenerator::smoothData(vtkSmartPointer<vtkParametricSpline> spline, vtkSmartPointer<vtkPolyData>& points,
                                       vtkSmartPointer<vtkPolyData>& derivatives)
{
  vtkSmartPointer<vtkPoints> new_points = vtkSmartPointer<vtkPoints>::New();

  vtkSmartPointer<vtkDoubleArray> derv = vtkSmartPointer<vtkDoubleArray>::New();
  derv->SetNumberOfComponents(3);


  // get points which are evenly spaced along the spline
  // initialize num_line_pts to some number, find the Euclidean distance between two points (m & n),
  // then use this distance to determine how many points should be used in the given line
  Eigen::Vector3d new_pt(0, 0, 0);
  long num_line_pts;
  double m[3], n[3];
  double u[3], pt[3], d[9]; // u - search point, pt - resulting point, d - unused but still required
  u[0] = u[1] = u[2] = 0;

  // computing entire line length
  double spline_length = 0;
  std::array<double, 3> p1, p2;
  spline->GetPoints()->GetPoint(0,p1.data());
  for(long i = 1; i < spline->GetPoints()->GetNumberOfPoints(); i++)
  {
    spline->GetPoints()->GetPoint(i,p2.data());
    spline_length += sqrt(vtk_viewer::pt_dist(p1.data(), p2.data()));
    std::copy(p2.begin(),p2.end(),p1.begin());
  }

  num_line_pts = static_cast<long>(std::ceil(spline_length/config_.point_spacing)) + 1;

  // Get points evenly spaced along the spline
  const double du = 1.0/(1.0*double(num_line_pts));
  for( int i = 0; i <= num_line_pts; ++i)
  {
    // Get point and store
    u[0] = i * du; // double(i)/double(num_line_pts);
    u[1] = i * du; // double(i)/double(num_line_pts);
    u[2] = i * du; // double(i)/double(num_line_pts);
    spline->Evaluate(u, pt, d);
    new_points->InsertNextPoint(pt);


    double pt1[3], pt2[3];
    // find nearby points in order to calculate the local derivative
    if(i == 0)
    {
      pt1[0] = pt[0];
      pt1[1] = pt[1];
      pt1[2] = pt[2];
      u[0] += du;
      spline->Evaluate(u, pt2, d);
    }
    else if(i == num_line_pts)
    {
      pt2[0] = pt[0];
      pt2[1] = pt[1];
      pt2[2] = pt[2];
      u[0] -= du;
      spline->Evaluate(u, pt1, d);
    }
    else
    {
      double u2[3] = {u[0], u[1], u[2]};
      u2[0] += du;
      spline->Evaluate(u2, pt2, d);

      u[0] -= du;
      spline->Evaluate(u, pt1, d);
    }

    // calculate the derivative and normalize
    new_pt[0] = pt1[0] - pt2[0];
    new_pt[1] = pt1[1] - pt2[1];
    new_pt[2] = pt1[2] - pt2[2];
    new_pt *= -1;
    new_pt.normalize();

    // insert the next derivative
    double new_ptr[3] = {new_pt[0], new_pt[1], new_pt[2]};
    derv->InsertNextTuple(&new_ptr[0]);
  }
  // Set points and normals
  points->SetPoints(new_points);
  computeSurfaceLineNormals(points);

  // Set points and derivatives
  derivatives->SetPoints(new_points);
  derivatives->GetPointData()->SetNormals(derv);
}

// Sort points in linear order
void SurfaceWalkRasterGenerator::sortPoints(vtkSmartPointer<vtkPoints>& points)
{
  std::vector<std::vector<double> > new_points;
  for(int i = 0; i < points->GetNumberOfPoints(); ++i)
  {
    double d[3];
    points->GetPoint(i, d);
    std::vector<double> pt;
    pt.push_back(d[0]);
    pt.push_back(d[1]);
    pt.push_back(d[2]);
    new_points.push_back(pt);
  }

  std::vector<std::vector<double> > sorted_points;

  // create new vector of sorted points
  std::size_t size = new_points.size();
  while(sorted_points.size() != size)
  {
    if(sorted_points.size() == 0)
    {
      sorted_points.push_back(new_points.front());
      new_points.erase(new_points.begin());
    }
    else
    {
      long next = findClosestPoint(sorted_points.back(), new_points);
      if (computeSquaredDistance(sorted_points.back(), new_points[static_cast<std::size_t>(next)]) < computeSquaredDistance(sorted_points.front(), new_points[static_cast<std::size_t>(next)]))
      {
        sorted_points.push_back(new_points[static_cast<std::size_t>(next)]);
        new_points.erase(new_points.begin() + next);
      }
      else
      {
        next = findClosestPoint(sorted_points.front(), new_points);
        sorted_points.insert(sorted_points.begin(), new_points[static_cast<std::size_t>(next)]);
        new_points.erase(new_points.begin() + next);
      }
    }
  }

  // convert array to vtkPoints and return
  vtkSmartPointer<vtkPoints> line_points = vtkSmartPointer<vtkPoints>::New();
  for(std::size_t i = 0; i < size; ++i)
  {
    line_points->InsertNextPoint(sorted_points[i][0], sorted_points[i][1], sorted_points[i][2]);
  }

  points = line_points;
}

bool SurfaceWalkRasterGenerator::computeSurfaceLineNormals(vtkSmartPointer<vtkPolyData>& data)
{
  // Find closest cell to each point and uses its normal vector
  vtkSmartPointer<vtkDoubleArray> new_norm = vtkSmartPointer<vtkDoubleArray>::New();
  new_norm->SetNumberOfComponents(3);
  new_norm->SetNumberOfTuples(data->GetPoints()->GetNumberOfPoints());

  for(int i = 0; i < data->GetPoints()->GetNumberOfPoints(); ++i)
  {

    // locate closest cell
    std::array<double,3> query_point;
    std::array<double,3> closest_point;
    vtkIdType cell_id;
    int sub_index;
    double dist;
    data->GetPoints()->GetPoint(i, query_point.data());
    cell_locator_->FindClosestPoint(query_point.data(),closest_point.data(),cell_id,sub_index,dist);
    if(cell_id < 0)
    {
      CONSOLE_BRIDGE_logError("FindClosestPoint returned an invalid cell id");
      return false;
    }

    // get normal
    Eigen::Vector3d normal_vect = Eigen::Vector3d::Zero();
    mesh_data_->GetCellData()->GetNormals()->GetTuple(cell_id,normal_vect.data());
    new_norm->SetTuple3(i,normal_vect(0),normal_vect(1),normal_vect(2));

  }
  // Insert the normal data
  data->GetPointData()->SetNormals(new_norm);
  return true;
}

vtkSmartPointer<vtkPolyData> SurfaceWalkRasterGenerator::createOffsetLine(vtkSmartPointer<vtkPolyData> line, vtkSmartPointer<vtkPolyData> derivatives, double dist)
{
  vtkSmartPointer<vtkPolyData> new_points;

  vtkDataArray* normals = line->GetPointData()->GetNormals();
  vtkDataArray* ders = derivatives->GetPointData()->GetNormals();

  // If normal or derivative data does not exist, or number of normals and derivatives do not match, return a null pointer
  if(!normals || !ders || normals->GetNumberOfTuples() != ders->GetNumberOfTuples())
  {
    CONSOLE_BRIDGE_logError("Could not create offset line");
    return new_points;
  }

  if(normals->GetNumberOfTuples() != line->GetNumberOfPoints())
  {
    CONSOLE_BRIDGE_logDebug("ERROR IN CALC OFFSET LINE");
    return new_points;
  }

  new_points = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> new_pts = vtkSmartPointer<vtkPoints>::New();

  // calculate offset for each point
  Eigen::Vector3d offset_dir;
  for(int i = 0; i < normals->GetNumberOfTuples(); ++i)
  {
    //calculate cross to get offset direction
    double* nrml = normals->GetTuple(i);
    double* line_dir = ders->GetTuple(i);

    Eigen::Vector3d u(nrml[0], nrml[1], nrml[2]);
    Eigen::Vector3d v(line_dir[0], line_dir[1], line_dir[2]);
    Eigen::Vector3d w = u.cross(v);
    w.normalize();

    if(i == 0)
    {
      offset_dir = w;
    }
    else
    {
      // check offset direction consistency to correct due to flipped normals
      double angle = computeAngle(offset_dir,w);
      if(angle > ANGLE_CORRECTION_THRESHOLD)
      {
        // flip direction of w
        w *= -1;
      }

      offset_dir = w;
    }

    // use point, direction w, and dist, to create new point
    double new_pt[3];
    double* pt = line->GetPoints()->GetPoint(i);
    new_pt[0] = pt[0] + w[0] * dist;
    new_pt[1] = pt[1] + w[1] * dist;
    new_pt[2] = pt[2] + w[2] * dist;

    new_pts->InsertNextPoint(new_pt);
  }

  // extrapolate end points to extend line beyond edges
  double new_pt[3];
  double* pt1;

  // Extend end point
  pt1 = new_pts->GetPoint(new_pts->GetNumberOfPoints()- 1);
  new_pt[0] = pt1[0];
  new_pt[1] = pt1[1];
  new_pt[2] = pt1[2];
  pt1 = new_pts->GetPoint(new_pts->GetNumberOfPoints()- 2);
  new_pt[0] +=  (new_pt[0] - pt1[0]);
  new_pt[1] +=  (new_pt[1] - pt1[1]);
  new_pt[2] +=  (new_pt[2] - pt1[2]);
  new_pts->SetPoint(new_pts->GetNumberOfPoints()-1, new_pt);

  // Extend front point
  pt1 = new_pts->GetPoint(0);
  new_pt[0] = pt1[0];
  new_pt[1] = pt1[1];
  new_pt[2] = pt1[2];
  pt1 = new_pts->GetPoint(1);
  new_pt[0] +=  (new_pt[0] - pt1[0]);
  new_pt[1] +=  (new_pt[1] - pt1[1]);
  new_pt[2] +=  (new_pt[2] - pt1[2]);
  new_pts->SetPoint(0, new_pt);

  new_points->SetPoints(new_pts);
  computeSurfaceLineNormals(new_points);

  return new_points;
}

vtkSmartPointer<vtkPolyData> SurfaceWalkRasterGenerator::extrudeSplineToSurface(vtkSmartPointer<vtkPolyData> line,
                                                                           double intersection_dist )
{
  vtkSmartPointer<vtkPolyData> new_surface;
  vtkSmartPointer<vtkDataArray> normals = line->GetPointData()->GetNormals();

  if(!normals)
  {
    CONSOLE_BRIDGE_logError("No normals, cannot create surface from spline");
    return new_surface;
  }

  new_surface = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  // for each point, insert 2 points, one above and one below, to create a new surface
  double dist_to_surf;
  Eigen::Vector3d closest_point;
  Eigen::Vector3d cell_normal;
  vtkIdType cell_id;
  int sub_index;
  for(int i = 0; i < line->GetNumberOfPoints(); ++i)
  {
    Eigen::Vector3d current_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_normal = Eigen::Vector3d::Zero();
    line->GetPoints()->GetPoint(i,current_point.data());
    line->GetPointData()->GetNormals()->GetTuple(i,current_normal.data());
    current_normal.normalize();

    // finding distance and closest point to surface
    Eigen::Vector3d ray_source_point = intersection_dist * current_normal + current_point;
    Eigen::Vector3d ray_target_point = -intersection_dist * current_normal + current_point;
    Eigen::Vector3d extruded_point_a, extruded_point_b, intersection_point;
    vtkSmartPointer<vtkPoints> intersection_points = vtkSmartPointer<vtkPoints>::New();
    int res = bsp_tree_->IntersectWithLine(ray_source_point.data(),ray_target_point.data(),
                                           RAY_INTERSECTION_TOLERANCE,
                                           intersection_points,nullptr);

    bool refine_extrude_points = false;
    Eigen::Vector3d dir;
    double dist = 0.0;
    if(res > 0)
    {
      intersection_points->GetPoint(0,intersection_point.data());
      dir = (intersection_point - current_point);
      dist = dir.norm();
      refine_extrude_points = dist > RAY_INTERSECTION_TOLERANCE;
    }

    if(!refine_extrude_points)
    {
      extruded_point_a = ray_source_point;
      extruded_point_b = ray_target_point;
    }
    else
    {
      // slightly past the current point opposite to the ray direction
      extruded_point_a = intersection_point - EXTRUDE_EXTEND_PERCENTAGE * dist * dir.normalized();

      // slightly past the intersection point in the ray direction
      extruded_point_b = current_point + EXTRUDE_EXTEND_PERCENTAGE * dist * dir.normalized();
    }

    if(i < line->GetNumberOfPoints() - 1)
    {
      vtkIdType start = i*2;

      cells->InsertNextCell(3);
      cells->InsertCellPoint(start);
      cells->InsertCellPoint(start+1);
      cells->InsertCellPoint(start+3);

      cells->InsertNextCell(3);
      cells->InsertCellPoint(start);
      cells->InsertCellPoint(start+3);
      cells->InsertCellPoint(start+2);
    }

    points->InsertNextPoint(extruded_point_a.data());
    points->InsertNextPoint(extruded_point_b.data());
  }

  new_surface->SetPolys(cells);
  new_surface->SetPoints(points);
  return new_surface;
}

vtkSmartPointer<vtkPolyData> SurfaceWalkRasterGenerator::createSurfaceFromSpline(vtkSmartPointer<vtkPolyData> line,
                                                                            double dist)
{
  vtkSmartPointer<vtkPolyData> new_surface;
  vtkSmartPointer<vtkDataArray> normals = line->GetPointData()->GetNormals();

  if(!normals)
  {
    CONSOLE_BRIDGE_logError("No normals, cannot create surface from spline");
    return new_surface;
  }

  new_surface = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  // for each point, insert 2 points, one above and one below, to create a new surface
  double dist_to_surf;
  for(int i = 0; i < line->GetNumberOfPoints(); ++i)
  {
    Eigen::Vector3d current_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_normal = Eigen::Vector3d::Zero();
    line->GetPoints()->GetPoint(i,current_point.data());
    line->GetPointData()->GetNormals()->GetTuple(i,current_normal.data());

    current_normal.normalize();
    Eigen::Vector3d point_below = -dist * current_normal  + current_point;
    Eigen::Vector3d point_above = dist * current_normal + current_point;
    if(i < line->GetNumberOfPoints() - 1)
    {
      vtkIdType start = i*2;

      cells->InsertNextCell(3);
      cells->InsertCellPoint(start);
      cells->InsertCellPoint(start+1);
      cells->InsertCellPoint(start+3);

      cells->InsertNextCell(3);
      cells->InsertCellPoint(start);
      cells->InsertCellPoint(start+3);
      cells->InsertCellPoint(start+2);
    }

    points->InsertNextPoint( point_above.data());
    points->InsertNextPoint(point_below.data());
  }

  new_surface->SetPolys(cells);
  new_surface->SetPoints(points);
  return new_surface;
}

} /* namespace tool_path_planner */

