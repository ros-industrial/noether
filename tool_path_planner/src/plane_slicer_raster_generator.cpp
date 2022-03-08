/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file plane_slicer_raster_generator.cpp
 * @date Dec 26, 2019
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tool_path_planner/plane_slicer_raster_generator.h>

#include <numeric>

#include <boost/make_shared.hpp>
#include <Eigen/StdVector>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkAppendPolyData.h>
#include <vtkBoundingBox.h>
#include <vtkCenterOfMass.h>
#include <vtkDoubleArray.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCutter.h>
#include <vtkErrorCode.h>
#include <vtkKdTree.h>
#include <vtkMath.h>
#include <vtkOBBTree.h>
#include <vtkParametricSpline.h>
#include <vtkPlane.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkStripper.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkSmartPointer.h>

#include <eigen_conversions/eigen_msg.h>
#include <console_bridge/console.h>

#include <noether_conversions/noether_conversions.h>
#include <tool_path_planner/utilities.h>

static const double EPSILON = 1e-6;

static Eigen::Vector3d getSegDir(vtkSmartPointer<vtkPolyData> seg)
{
  if (seg->GetPoints()->GetNumberOfPoints() < 1)
  {
    CONSOLE_BRIDGE_logError("can't get direction from a segment with fewer than 2 points");
    Eigen::Vector3d v;
    v.x() = 1.0;
    v.y() = 0.0;
    v.z() = 0.0;
    return (v);
  }
  Eigen::Vector3d seg_start, seg_end;
  seg->GetPoint(0, seg_start.data());
  seg->GetPoint(seg->GetPoints()->GetNumberOfPoints() - 1, seg_end.data());
  return (seg_end - seg_start);
}

static bool compare_ds_pair(std::pair<double, size_t>& first, std::pair<double, size_t>& second)
{
  return (first.first < second.first);
}

// @brief this function accepts and returns a rasterConstruct where every segment progresses in the same direction and
// in the right order
static tool_path_planner::PlaneSlicerRasterGenerator::RasterConstructData
alignRasterCD(tool_path_planner::PlaneSlicerRasterGenerator::RasterConstructData& rcd,
              Eigen::Vector3d& raster_direction)
{
  if (rcd.raster_segments[0]->GetPoints()->GetNumberOfPoints() <= 1)
  {
    CONSOLE_BRIDGE_logError("first raster segment has 0 or 1 points, unable to alignRasterCD()");
    return (rcd);
  }
  Eigen::Vector3d raster_start;
  rcd.raster_segments[0]->GetPoint(0, raster_start.data());

  // determine location and direction of each successive segement, reverse any mis-directed segments
  tool_path_planner::PlaneSlicerRasterGenerator::RasterConstructData temp_rcd;
  std::list<std::pair<double, size_t> > seg_order;  // once sorted this list will be the order of the segments
  for (size_t i = 0; i < rcd.raster_segments.size(); i++)
  {
    // determine i'th segments direction
    Eigen::Vector3d seg_dir = getSegDir(rcd.raster_segments[i]);
    Eigen::Vector3d seg_start;
    rcd.raster_segments[i]->GetPoint(0, seg_start.data());
    // if segment direction is opposite raster direction, reverse the segment
    if (seg_dir.dot(raster_direction) < 0.0)
    {
      vtkSmartPointer<vtkPoints> old_points = rcd.raster_segments[i]->GetPoints();
      vtkSmartPointer<vtkPoints> new_points = vtkSmartPointer<vtkPoints>::New();
      for (long int j = static_cast<long int>(old_points->GetNumberOfPoints() - 1); j >= 0; j--)
      {
        new_points->InsertNextPoint(old_points->GetPoint(j));
      }
      rcd.raster_segments[i]->SetPoints(new_points);
      rcd.raster_segments[i]->GetPoint(0, seg_start.data());
    }

    // determine location of this segment in raster
    double seg_loc = (seg_start - raster_start).dot(raster_direction);
    std::pair<double, size_t> p(seg_loc, i);
    seg_order.push_back(p);
  }
  // sort the segments by location
  if (seg_order.size() >= 1)
  {
    seg_order.sort(compare_ds_pair);
  }
  tool_path_planner::PlaneSlicerRasterGenerator::RasterConstructData new_rcd;
  for (std::pair<double, size_t> p : seg_order)
  {
    size_t seg_index = std::get<1>(p);
    new_rcd.raster_segments.push_back(rcd.raster_segments[seg_index]);
    new_rcd.segment_lengths.push_back(rcd.segment_lengths[seg_index]);
  }
  return (new_rcd);
}

static Eigen::Matrix3d computeRotation(const Eigen::Vector3d& vx, const Eigen::Vector3d& vy, const Eigen::Vector3d& vz)
{
  Eigen::Matrix3d m;
  m.setIdentity();
  m.col(0) = vx.normalized();
  m.col(1) = vy.normalized();
  m.col(2) = vz.normalized();
  return m;
}

static vtkSmartPointer<vtkTransform> toVtkMatrix(const Eigen::Affine3d& t)
{
  vtkSmartPointer<vtkTransform> vtk_t = vtkSmartPointer<vtkTransform>::New();
  vtk_t->PreMultiply();
  vtk_t->Identity();
  vtk_t->Translate(t.translation().x(), t.translation().y(), t.translation().z());
  Eigen::Vector3d rpy = t.rotation().eulerAngles(0, 1, 2);
  vtk_t->RotateX(vtkMath::DegreesFromRadians(rpy(0)));
  vtk_t->RotateY(vtkMath::DegreesFromRadians(rpy(1)));
  vtk_t->RotateZ(vtkMath::DegreesFromRadians(rpy(2)));
  vtk_t->Scale(1.0, 1.0, 1.0);
  return vtk_t;
}

double computeLength(const vtkSmartPointer<vtkPoints>& points)
{
  const vtkIdType num_points = points->GetNumberOfPoints();
  double total_length = 0.0;
  if (num_points < 2)
  {
    return total_length;
  }

  Eigen::Vector3d p0, pf;
  for (vtkIdType i = 1; i < num_points; i++)
  {
    points->GetPoint(i - 1, p0.data());
    points->GetPoint(i, pf.data());

    total_length += (pf - p0).norm();
  }
  return total_length;
}

static vtkSmartPointer<vtkPoints> applyParametricSpline(const vtkSmartPointer<vtkPoints>& points,
                                                        double total_length,
                                                        double point_spacing)
{
  vtkSmartPointer<vtkPoints> new_points = vtkSmartPointer<vtkPoints>::New();

  // create spline
  vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
  spline->SetPoints(points);
  spline->SetParameterizeByLength(true);
  spline->ClosedOff();

  // adding first point
  Eigen::Vector3d pt_prev;
  points->GetPoint(0, pt_prev.data());
  new_points->InsertNextPoint(pt_prev.data());

  // adding remaining points by evaluating spline
  std::size_t num_points = static_cast<std::size_t>(std::ceil(total_length / point_spacing) + 1);
  double du[9];
  Eigen::Vector3d u, pt;
  for (unsigned i = 1; i < num_points; i++)
  {
    double interv = static_cast<double>(i) / static_cast<double>(num_points - 1);
    interv = interv > 1.0 ? 1.0 : interv;
    if (std::abs(interv - 1.0) < EPSILON)
    {
      break;  // reach end
    }

    u = interv * Eigen::Vector3d::Ones();
    std::tie(u[0], u[1], u[2]) = std::make_tuple(interv, interv, interv);
    spline->Evaluate(u.data(), pt.data(), du);

    // check distance
    if (1)
    // TODO Figure out why this reduces the number of points by half and makes them twice the point spacing
    //    if ((pt - pt_prev).norm() >= point_spacing)
    {
      new_points->InsertNextPoint(pt.data());
      pt_prev = pt;
    }
  }

  // add last point
  points->GetPoint(points->GetNumberOfPoints() - 1, pt_prev.data());
  new_points->InsertNextPoint(pt_prev.data());

  return new_points;
}

/**
 * @brief removes points that appear in multiple lists such that only one instance of that point
 *        index remains
 * @param points_lists
 */
static void removeRedundant(std::vector<std::vector<vtkIdType> >& points_lists)
{
  using IdList = std::vector<vtkIdType>;
  if (points_lists.size() < 2)
  {
    return;
  }

  std::vector<std::vector<vtkIdType> > new_points_lists;
  new_points_lists.push_back(points_lists.front());
  for (std::size_t i = 1; i < points_lists.size(); i++)
  {
    IdList& current_list = points_lists[i];
    IdList new_list;
    IdList all_ids;

    // create list of all ids
    for (auto& ref_list : new_points_lists)
    {
      all_ids.insert(all_ids.end(), ref_list.begin(), ref_list.end());
    }

    for (auto& id : current_list)
    {
      // add if not found in any of the previous lists
      if (std::find(all_ids.begin(), all_ids.end(), id) == all_ids.end())
      {
        new_list.push_back(id);
      }
    }

    // add if it has enough points
    if (new_list.size() > 0)
    {
      new_points_lists.push_back(new_list);
    }
  }

  points_lists.clear();
  points_lists.assign(new_points_lists.begin(), new_points_lists.end());
}

static void mergeRasterSegments(const vtkSmartPointer<vtkPoints>& points,
                                double merge_dist,
                                std::vector<std::vector<vtkIdType> >& points_lists)
{
  using namespace Eigen;
  using IdList = std::vector<vtkIdType>;
  if (points_lists.size() < 2)
  {
    return;
  }

  std::vector<IdList> new_points_lists;
  IdList merged_list_ids;
  IdList merged_list;

  auto do_merge =
      [&points](const IdList& current_list, const IdList& next_list, double merge_dist, IdList& merged_list) {
        Vector3d cl_point, nl_point;

        // checking front and back end points respectively
        points->GetPoint(current_list.front(), cl_point.data());
        points->GetPoint(next_list.back(), nl_point.data());
        double d = (cl_point - nl_point).norm();
        if (d < merge_dist)
        {
          merged_list.assign(next_list.begin(), next_list.end());
          merged_list.insert(merged_list.end(), current_list.begin(), current_list.end());
          return true;
        }

        // checking back and front end points respectively
        points->GetPoint(current_list.back(), cl_point.data());
        points->GetPoint(next_list.front(), nl_point.data());
        d = (cl_point - nl_point).norm();
        if (d < merge_dist)
        {
          merged_list.assign(current_list.begin(), current_list.end());
          merged_list.insert(merged_list.end(), next_list.begin(), next_list.end());
          return true;
        }
        return false;
      };

  for (std::size_t i = 0; i < points_lists.size(); i++)
  {
    if (std::find(merged_list_ids.begin(), merged_list_ids.end(), i) != merged_list_ids.end())
    {
      // already merged
      CONSOLE_BRIDGE_logDebug("Segment %i has already been merged, skipping", i);
      continue;
    }

    IdList current_list = points_lists[i];
    Vector3d cl_point, nl_point;
    bool seek_adjacent = true;
    while (seek_adjacent)
    {
      seek_adjacent = false;
      for (std::size_t j = i + 1; j < points_lists.size(); j++)
      {
        if (std::find(merged_list_ids.begin(), merged_list_ids.end(), j) != merged_list_ids.end())
        {
          // already merged
          CONSOLE_BRIDGE_logDebug("Segment %i has already been merged, skipping", j);
          continue;
        }

        merged_list.clear();
        IdList next_list = points_lists[j];
        if (do_merge(current_list, next_list, merge_dist, merged_list))
        {
          CONSOLE_BRIDGE_logDebug("Merged segment %lu onto segment %lu", j, i);
          current_list = merged_list;
          merged_list_ids.push_back(static_cast<vtkIdType>(j));
          seek_adjacent = true;
          continue;
        }

        std::reverse(next_list.begin(), next_list.end());
        if (do_merge(current_list, next_list, merge_dist, merged_list))
        {
          CONSOLE_BRIDGE_logDebug("Merged segment %lu onto segment %lu", j, i);
          current_list = merged_list;
          merged_list_ids.push_back(static_cast<vtkIdType>(j));
          seek_adjacent = true;
          continue;
        }
      }
    }
    new_points_lists.push_back(current_list);
  }
  points_lists.clear();
  std::copy_if(new_points_lists.begin(), new_points_lists.end(), std::back_inserter(points_lists), [](const IdList& l) {
    return l.size() > 1;
  });
  CONSOLE_BRIDGE_logDebug("Final raster contains %lu segments", points_lists.size());
}

static void rectifyDirection(const vtkSmartPointer<vtkPoints>& points,
                             const Eigen::Vector3d& ref_point,
                             std::vector<std::vector<vtkIdType> >& points_lists)
{
  using namespace Eigen;
  Vector3d p0, pf;
  if (points_lists.empty())
  {
    return;
  }

  // getting first and last points
  points->GetPoint(points_lists.front().front(), p0.data());
  points->GetPoint(points_lists.back().back(), pf.data());

  bool reverse = (ref_point - p0).norm() > (ref_point - pf).norm();
  if (reverse)
  {
    for (auto& s : points_lists)  // reverse points in segments
    {
      std::reverse(s.begin(), s.end());
    }
    std::reverse(points_lists.begin(), points_lists.end());
  }
}

namespace tool_path_planner
{
void PlaneSlicerRasterGenerator::setConfiguration(const PlaneSlicerRasterGenerator::Config& config)
{
  config_ = config;
}

void PlaneSlicerRasterGenerator::setInput(vtkSmartPointer<vtkPolyData> mesh)
{
  if (!mesh_data_)
    mesh_data_ = vtkSmartPointer<vtkPolyData>::New();

  mesh_data_->DeepCopy(mesh);

  if (mesh_data_->GetPointData()->GetNormals() && mesh_data_->GetCellData()->GetNormals())
  {
    CONSOLE_BRIDGE_logInform("Normal data is available", getName().c_str());
  }
  else
  {
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

    if (!mesh_data_->GetPointData()->GetNormals())
    {
      mesh_data_->GetPointData()->SetNormals(normal_generator->GetOutput()->GetPointData()->GetNormals());
    }

    if (!mesh_data_->GetCellData()->GetNormals())
    {
      mesh_data_->GetCellData()->SetNormals(normal_generator->GetOutput()->GetCellData()->GetNormals());
    }
  }
}

void PlaneSlicerRasterGenerator::setInput(pcl::PolygonMesh::ConstPtr mesh)
{
  auto mesh_data = vtkSmartPointer<vtkPolyData>::New();
  pcl::VTKUtils::mesh2vtk(*mesh, mesh_data);
  mesh_data->BuildLinks();
  mesh_data->BuildCells();

  // compute face and vertex using polygon info
  tool_path_planner::computePCLMeshNormals(mesh, face_normals_, vertex_normals_);

  // compute vertex normals using Moving Least Squares
  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromPCLPointCloud2(mesh->cloud, *mesh_cloud_ptr);
  mls_mesh_normals_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointNormal> >(
      tool_path_planner::computeMLSMeshNormals(mesh_cloud_ptr, config_.search_radius));

  // align mls_vertex_normals to vertex_normals
  if (!tool_path_planner::alignToVertexNormals(*mls_mesh_normals_ptr_, vertex_normals_))
  {
    CONSOLE_BRIDGE_logError("alignToVertexNormals failed");
  }

  setInput(mesh_data);
}

void PlaneSlicerRasterGenerator::setInput(const shape_msgs::Mesh& mesh)
{
  pcl::PolygonMesh::Ptr pcl_mesh = boost::make_shared<pcl::PolygonMesh>();
  noether_conversions::convertToPCLMesh(mesh, *pcl_mesh);
  setInput(pcl_mesh);
}

vtkSmartPointer<vtkPolyData> PlaneSlicerRasterGenerator::getInput() { return mesh_data_; }
void PlaneSlicerRasterGenerator::computePoseData(const vtkSmartPointer<vtkPolyData>& polydata,
                                                 int idx,
                                                 Eigen::Vector3d& p,
                                                 Eigen::Vector3d& vx,
                                                 Eigen::Vector3d& vy,
                                                 Eigen::Vector3d& vz)
{
  Eigen::Vector3d p_next;
  Eigen::Vector3d p_start;

  //  polydata->GetPoint(idx, p.data()); // use this and next point to determine x direction
  //  polydata->GetPoint(idx + 1, p_next.data());
  polydata->GetPoint(0, p_start.data());  // use the first and last point to determine x direction
  polydata->GetPoint(idx, p.data());
  polydata->GetPoint(polydata->GetNumberOfPoints() - 1, p_next.data());
  polydata->GetPointData()->GetNormals()->GetTuple(idx, vz.data());
  vz = vz.normalized();
  vx = p_next - p_start;
  vx = (vx - vx.dot(vz) * vz).normalized();
  vy = vz.cross(vx).normalized();
}

tool_path_planner::ToolPaths PlaneSlicerRasterGenerator::convertToPoses(
    const std::vector<tool_path_planner::PlaneSlicerRasterGenerator::RasterConstructData>& rasters_data)
{
  using namespace Eigen;
  tool_path_planner::ToolPaths rasters_array;
  for (const tool_path_planner::PlaneSlicerRasterGenerator::RasterConstructData& rd : rasters_data)  // for every raster
  {
    tool_path_planner::ToolPath raster_path;
    std::vector<vtkSmartPointer<vtkPolyData>> raster_segments;
    raster_segments.assign(rd.raster_segments.begin(), rd.raster_segments.end());
    for (const vtkSmartPointer<vtkPolyData>& polydata : raster_segments)  // for every segment
    {
      tool_path_planner::ToolPathSegment raster_path_segment;
      std::size_t num_points = polydata->GetNumberOfPoints();
      Vector3d p, next_p;
      Vector3d prev_vx, prev_vy, prev_vz;
      Vector3d vx, vy, vz;
      Vector3d next_vx, next_vy, next_vz;
      Isometry3d pose;
      std::vector<int> indices(num_points);
      std::iota(indices.begin(), indices.end(), 0);
      // for every waypoint MAKE A POSE such that
      // its normal uses the mesh normal
      // its x axis points along path
      // its y is z cross x via right hand rule
      // average 3 x-vectors to smooth out chatter caused by triangles
      computePoseData(polydata, 0, p, vx, vy, vz);
      prev_vx = vx;
      prev_vy = vy;
      prev_vz = vz;
      int q = 0;
      for (std::size_t i = 0; i < indices.size() - 2; i++)
      {
        computePoseData(polydata, i + 1, next_p, next_vx, next_vy, next_vz);
        vx = (prev_vx + vx + next_vx).normalized();
        vy = vz.cross(vx).normalized();
        pose = Translation3d(p) * AngleAxisd(computeRotation(vx, vy, vz));
        raster_path_segment.push_back(pose);
        q++;

        prev_vx = vx;
        prev_vy = vy;
        prev_vz = vz;
        vx = next_vx;
        vy = next_vy;
        vz = next_vz;
        p = next_p;
      }  // end for every waypoint

      if (indices.size() >= 2)  // this throws away short segments
      {
        // adding next to last pose
        computePoseData(polydata, indices.size() - 2, p, vx, vy, vz);
        vx = (prev_vx + vx).normalized();
        pose = Translation3d(p) * AngleAxisd(computeRotation(vx, vy, vz));
        raster_path_segment.push_back(pose);
        q++;

        // adding last pose
        polydata->GetPoint(indices.size() - 1, p.data());
        pose = Translation3d(p) * AngleAxisd(computeRotation(vx, vy, vz));
        raster_path_segment.push_back(pose);
        q++;
      }
      raster_path.push_back(raster_path_segment);
    }  // end for every segment
    rasters_array.push_back(raster_path);
  }  // end for every raster

  return rasters_array;
}

boost::optional<ToolPaths> PlaneSlicerRasterGenerator::generate()
{
  using namespace Eigen;
  using IDVec = std::vector<vtkIdType>;

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_ERROR);

  if (!mesh_data_)
  {
    CONSOLE_BRIDGE_logDebug("%s No mesh data has been provided", getName().c_str());
  }
  // Assign the longest axis of the bounding box to x, middle to y, and shortest to z.
  Vector3d corner, x_dir, y_dir, z_dir, sizes;

  if (config_.raster_wrt_global_axes)
  {
    // Determine extent of mesh along axes of current coordinate frame
    VectorXd bounds(6);
    std::vector<Vector3d> extents;
    mesh_data_->GetBounds(bounds.data());
    extents.push_back(Vector3d::UnitX() * (bounds[1] - bounds[0]));  // Extent in x-direction of supplied mesh
                                                                     // coordinate frame
    extents.push_back(Vector3d::UnitY() * (bounds[3] - bounds[2]));  // Extent in y-direction of supplied mesh
                                                                     // coordinate frame
    extents.push_back(Vector3d::UnitZ() * (bounds[5] - bounds[4]));  // Extent in z-direction of supplied mesh
                                                                     // coordinate frame

    // find min and max magnitude.
    int max = 0;
    int min = 0;
    for (std::size_t i = 1; i < extents.size(); ++i)
    {
      if (extents[max].squaredNorm() < extents[i].squaredNorm())
        max = i;
      else if (extents[min].squaredNorm() > extents[i].squaredNorm())
        min = i;
    }

    // Order the axes.  Computing y saves comparisons and guarantees right-handedness.
    Eigen::Matrix3d rotation_transform;
    rotation_transform.col(0) = extents[max].normalized();
    rotation_transform.col(2) = extents[min].normalized();
    rotation_transform.col(1) = rotation_transform.col(2).cross(rotation_transform.col(0)).normalized();

    // Extract our axes transfored to align to the major axes of our aabb
    x_dir = rotation_transform.row(0).normalized();
    y_dir = rotation_transform.row(1).normalized();
    z_dir = rotation_transform.row(2).normalized();
  }
  else
  {
    // computing major axes using oob and assign to x_dir, y_dir, z_dir
    vtkSmartPointer<vtkOBBTree> oob = vtkSmartPointer<vtkOBBTree>::New();
    oob->ComputeOBB(mesh_data_, corner.data(), x_dir.data(), y_dir.data(), z_dir.data(), sizes.data());
  }

  // Compute the center of mass
  Vector3d origin;
  vtkSmartPointer<vtkCenterOfMass> cog_filter = vtkSmartPointer<vtkCenterOfMass>::New();
  cog_filter->SetInputData(mesh_data_);
  cog_filter->SetUseScalarsAsWeights(false);
  cog_filter->Update();
  cog_filter->GetCenter(origin.data());

  // computing transformation matrix
  Affine3d t = Translation3d(origin) * AngleAxisd(computeRotation(x_dir, y_dir, z_dir));

  // transforming data
  vtkSmartPointer<vtkTransform> vtk_transform = toVtkMatrix(t);

  vtkSmartPointer<vtkTransformFilter> transform_filter = vtkSmartPointer<vtkTransformFilter>::New();
  transform_filter->SetInputData(mesh_data_);
  transform_filter->SetTransform(vtk_transform);
  transform_filter->Update();
  vtkSmartPointer<vtkPolyData> transformed_mesh_data = transform_filter->GetPolyDataOutput();

  // compute bounds
  VectorXd bounds(6);
  Vector3d center;
  Vector3d half_ext;
  transformed_mesh_data->GetBounds(bounds.data());

  // calculating size
  sizes.x() = bounds[1] - bounds[0];
  sizes.y() = bounds[3] - bounds[2];
  sizes.z() = bounds[5] - bounds[4];

  half_ext = sizes / 2.0;
  center = Eigen::Vector3d(bounds[0], bounds[2], bounds[4]) + half_ext;

  // Apply the rotation offset about the short direction (new Z axis) of the bounding box
  Isometry3d rotation_offset = Isometry3d::Identity() * AngleAxisd(config_.raster_rot_offset, Vector3d::UnitZ());

  // Calculate direction of raster strokes, rotated by the above-specified amount
  Vector3d raster_dir;
  if (config_.raster_direction.isApprox(Eigen::Vector3d::Zero()))
  {
    CONSOLE_BRIDGE_logError("APPROX ZERO");
    // If no direction was specified, use the middle dimension of the bounding box
    raster_dir = Eigen::Vector3d::UnitY();
    raster_dir = (rotation_offset * raster_dir).normalized();
  }
  else
  {
    // If a direction was specified, transform it into the frame of the bounding box
    raster_dir = (rotation_offset *                                   // Rotation about short axis of bounding box
                  AngleAxisd(computeRotation(x_dir, y_dir, z_dir)) *  // Rotation part of 't' (recalculated because
                                                                      // Eigen makes it hard to access)
                  config_.raster_direction)                           // Raster direction specified by user
                     .normalized();
  }

  // Calculate all 8 corners projected onto the raster direction vector
  Eigen::VectorXd dist(8);
  dist(0) = raster_dir.dot(half_ext);
  dist(1) = raster_dir.dot(Eigen::Vector3d(half_ext.x(), -half_ext.y(), half_ext.z()));
  dist(2) = raster_dir.dot(Eigen::Vector3d(half_ext.x(), -half_ext.y(), -half_ext.z()));
  dist(3) = raster_dir.dot(Eigen::Vector3d(half_ext.x(), half_ext.y(), -half_ext.z()));
  dist(4) = raster_dir.dot(-half_ext);
  dist(5) = raster_dir.dot(Eigen::Vector3d(-half_ext.x(), -half_ext.y(), half_ext.z()));
  dist(6) = raster_dir.dot(Eigen::Vector3d(-half_ext.x(), -half_ext.y(), -half_ext.z()));
  dist(7) = raster_dir.dot(Eigen::Vector3d(-half_ext.x(), half_ext.y(), -half_ext.z()));

  double max_coeff = dist.maxCoeff();
  double min_coeff = dist.minCoeff();

  // Calculate the number of planes to cover the bounding box along the direction vector
  auto num_planes = static_cast<std::size_t>(std::ceil((max_coeff - min_coeff) / config_.raster_spacing));

  // Calculate the start location
  Vector3d start_loc = center + (min_coeff * raster_dir);
  vtkSmartPointer<vtkAppendPolyData> raster_data = vtkSmartPointer<vtkAppendPolyData>::New();
  for (std::size_t i = 0; i < num_planes + 1; i++)
  {
    vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
    vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
    vtkSmartPointer<vtkStripper> stripper = vtkSmartPointer<vtkStripper>::New();

    Vector3d current_loc = start_loc + i * config_.raster_spacing * raster_dir;
    plane->SetOrigin(current_loc.x(), current_loc.y(), current_loc.z());
    plane->SetNormal(raster_dir.x(), raster_dir.y(), raster_dir.z());

    cutter->SetCutFunction(plane);
    cutter->SetInputData(transformed_mesh_data);
    cutter->SetSortBy(1);
    cutter->SetGenerateTriangles(false);
    cutter->Update();

    stripper->SetInputConnection(cutter->GetOutputPort());
    stripper->JoinContiguousSegmentsOn();
    stripper->SetMaximumLength(mesh_data_->GetNumberOfPoints());
    stripper->Update();

    if (stripper->GetErrorCode() != vtkErrorCode::NoError)
    {
      continue;
    }

    for (int r = 0; r < stripper->GetNumberOfOutputPorts(); r++)
    {
      raster_data->AddInputData(stripper->GetOutput(r));
    }
  }
  // build cell locator and kd_tree to recover normals later on
  kd_tree_ = vtkSmartPointer<vtkKdTreePointLocator>::New();
  kd_tree_->SetDataSet(mesh_data_);
  kd_tree_->BuildLocator();

  cell_locator_ = vtkSmartPointer<vtkCellLocator>::New();
  cell_locator_->SetDataSet(mesh_data_);
  cell_locator_->BuildLocator();

  // collect rasters and set direction
  raster_data->Update();
  vtkIdType num_slices = raster_data->GetTotalNumberOfInputConnections();
  std::vector<tool_path_planner::PlaneSlicerRasterGenerator::RasterConstructData> rasters_data_vec;
  std::vector<IDVec> raster_ids;
  boost::optional<Vector3d> ref_dir;
  for (std::size_t i = 0; i < num_slices; i++)
  {
    tool_path_planner::PlaneSlicerRasterGenerator::RasterConstructData r;

    // collecting raster segments based on min hole size
    vtkSmartPointer<vtkPolyData> raster_lines = raster_data->GetInput(i);
    vtkIdType* indices;
    vtkIdType num_points;
    vtkIdType num_lines = raster_lines->GetNumberOfLines();
    vtkCellArray* cells = raster_lines->GetLines();

    CONSOLE_BRIDGE_logDebug("%s raster %i has %i lines and %i points",
                            getName().c_str(),
                            i,
                            raster_lines->GetNumberOfLines(),
                            raster_lines->GetNumberOfPoints());

    if (num_lines == 0)
    {
      continue;
    }

    raster_ids.clear();

    unsigned int lineCount = 0;
    for (cells->InitTraversal(); cells->GetNextCell(num_points, indices); lineCount++)
    {
      IDVec point_ids;
      for (vtkIdType i = 0; i < num_points; i++)
      {
        if (std::find(point_ids.begin(), point_ids.end(), indices[i]) != point_ids.end())
        {
          continue;
        }
        point_ids.push_back(indices[i]);
      }
      if (point_ids.empty())
      {
        continue;
      }

      // removing duplicates
      auto iter = std::unique(point_ids.begin(), point_ids.end());
      point_ids.erase(iter, point_ids.end());

      // adding
      raster_ids.push_back(point_ids);
    }

    if (raster_ids.empty())
    {
      continue;
    }

    // remove redundant indices
    removeRedundant(raster_ids);

    // merging segments
    mergeRasterSegments(raster_lines->GetPoints(), config_.min_hole_size, raster_ids);

    // rectifying
    if (!rasters_data_vec.empty() && !rasters_data_vec.back().raster_segments.empty())
    {
      Vector3d ref_point;
      rasters_data_vec.back().raster_segments.front()->GetPoint(0, ref_point.data());  // first point in previous raster
      rectifyDirection(raster_lines->GetPoints(), t * ref_point, raster_ids);
    }

    for (auto& rpoint_ids : raster_ids)
    {
      // Populating with points
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
      std::for_each(rpoint_ids.begin(), rpoint_ids.end(), [&points, &raster_lines](vtkIdType& id) {
        std::array<double, 3> p;
        raster_lines->GetPoint(id, p.data());
        points->InsertNextPoint(p.data());
      });

      // compute length and add points if segment length is greater than threshold
      double line_length = computeLength(points);
      if (line_length > config_.min_segment_size && points->GetNumberOfPoints() > 1)
      {
        // enforce point spacing
        decltype(points) new_points = applyParametricSpline(points, line_length, config_.point_spacing);

        // add points to segment now
        vtkSmartPointer<vtkPolyData> segment_data = vtkSmartPointer<vtkPolyData>::New();
        segment_data->SetPoints(new_points);

        // transforming to original coordinate system
        transform_filter = vtkSmartPointer<vtkTransformFilter>::New();
        transform_filter->SetInputData(segment_data);
        transform_filter->SetTransform(toVtkMatrix(t.inverse()));
        transform_filter->Update();
        segment_data = transform_filter->GetPolyDataOutput();

        // inserting normals
        if (!insertNormals(config_.search_radius, segment_data))
        {
          CONSOLE_BRIDGE_logError("%s failed to insert normals to segment %lu of raster %lu",
                                  getName().c_str(),
                                  r.raster_segments.size(),
                                  i);
          return boost::none;
        }

        // saving into raster
        if (segment_data->GetPoints()->GetNumberOfPoints() > 0)
        {
          r.raster_segments.push_back(segment_data);
          r.segment_lengths.push_back(line_length);
        }
      }
    }

    if (r.raster_segments.size() > 0)
      rasters_data_vec.push_back(r);
  }
  if (rasters_data_vec.size() == 0)
  {
    CONSOLE_BRIDGE_logError("no rasters found");
    ToolPaths rasters;
    return (rasters);
  }
  // make sure every raster has its segments ordered and aligned correctly
  Eigen::Vector3d raster_direction = getSegDir(rasters_data_vec[0].raster_segments[0]);
  for (tool_path_planner::PlaneSlicerRasterGenerator::RasterConstructData rcd : rasters_data_vec)
  {
    rcd = alignRasterCD(rcd, raster_direction);
  }

  if (config_.interleave_rasters)
  {
    std::vector<tool_path_planner::PlaneSlicerRasterGenerator::RasterConstructData> tmp_rasters_data_vec;
    // evens
    for (size_t i = 0; i < rasters_data_vec.size(); i += 2)
    {
      tmp_rasters_data_vec.push_back(rasters_data_vec[i]);
    }
    // odds
    for (size_t i = 1; i < rasters_data_vec.size(); i += 2)
    {
      tmp_rasters_data_vec.push_back(rasters_data_vec[i]);
    }
    // clear and copy new order
    rasters_data_vec.clear();
    for (size_t i = 0; i < tmp_rasters_data_vec.size(); i++)
    {
      rasters_data_vec.push_back(tmp_rasters_data_vec[i]);
    }
  }

  // converting to poses msg
  ToolPaths rasters = convertToPoses(rasters_data_vec);

  if (config_.generate_extra_rasters)
  {
    rasters = addExtraWaypoints(rasters, config_.raster_spacing, config_.point_spacing);
  }

  if ((config_.raster_style != PROCESS_FORWARD_DIRECTION_ONLY))
  {
    // switch directions of every other raster, using either flipping orientation or not depending on style selected
    rasters = reverseOddRasters(rasters, config_.raster_style);
  }
  return rasters;
}

bool PlaneSlicerRasterGenerator::insertNormals(const double search_radius, vtkSmartPointer<vtkPolyData>& data)
{
  // Find closest cell to each point and uses its normal vector
  vtkSmartPointer<vtkDoubleArray> new_normals = vtkSmartPointer<vtkDoubleArray>::New();
  new_normals->SetNumberOfComponents(3);
  new_normals->SetNumberOfTuples(data->GetPoints()->GetNumberOfPoints());

  // get normal data
  vtkSmartPointer<vtkDataArray> normal_data = mesh_data_->GetPointData()->GetNormals();

  if (!normal_data)
  {
    CONSOLE_BRIDGE_logError("%s Normal data is not available", getName().c_str());
    return false;
  }

  Eigen::Vector3d normal_vect = Eigen::Vector3d::UnitZ();
  for (int i = 0; i < data->GetPoints()->GetNumberOfPoints(); ++i)
  {
    // locate closest cell
    Eigen::Vector3d query_point;
    vtkSmartPointer<vtkIdList> id_list = vtkSmartPointer<vtkIdList>::New();
    data->GetPoints()->GetPoint(i, query_point.data());
    kd_tree_->FindClosestNPoints(1, query_point.data(), id_list);
    if (id_list->GetNumberOfIds() < 1)
    {
      CONSOLE_BRIDGE_logError("%s failed to find closest for normal computation", getName().c_str());
      return false;
    }

    // compute normal average
    normal_vect = Eigen::Vector3d::Zero();
    vtkIdType p_id = id_list->GetId(0);
    if (p_id < 0)
    {
      CONSOLE_BRIDGE_logError("%s point id is invalid", getName().c_str());
      continue;
    }

    normal_vect.x() = mls_mesh_normals_ptr_->points[p_id].normal_x;
    normal_vect.y() = mls_mesh_normals_ptr_->points[p_id].normal_y;
    normal_vect.z() = mls_mesh_normals_ptr_->points[p_id].normal_z;
    normal_vect.normalize();

    // save normal
    new_normals->SetTuple3(i, normal_vect(0), normal_vect(1), normal_vect(2));
  }  // end for every point

  data->GetPointData()->SetNormals(new_normals);
  return true;
}

std::string PlaneSlicerRasterGenerator::getName() const { return getClassName<decltype(*this)>(); }

} /* namespace tool_path_planner */
