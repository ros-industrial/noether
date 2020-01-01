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

#include <vtkMath.h>
#include <vtkBoundingBox.h>
#include <vtkDoubleArray.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkOBBTree.h>
#include <vtkKdTree.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkCutter.h>
#include <vtkStripper.h>
#include <vtkPlane.h>
#include <vtkAppendPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkCenterOfMass.h>
#include <vtkParametricSpline.h>
#include <vtkPolyDataNormals.h>
#include <vtkErrorCode.h>

#include <boost/make_shared.hpp>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <console_bridge/console.h>
#include <noether_conversions/noether_conversions.h>
#include "tool_path_planner/utilities.h"
#include "tool_path_planner/plane_slicer_raster_generator.h"

static const double EPSILON = 1e-6;

using PolyDataPtr = vtkSmartPointer<vtkPolyData>;
struct RasterConstructData
{
  std::vector<PolyDataPtr> raster_segments;
  std::vector<double> segment_lengths;
  bool reverse_direction = false;
};

static Eigen::Matrix3d computeRotation(const Eigen::Vector3d& vx, const Eigen::Vector3d& vy, const Eigen::Vector3d& vz)
{
  Eigen::Matrix3d m;
  m.setIdentity();
  m.col(0) = vx.normalized();
  m.col(1) = vy.normalized();
  m.col(2) = vz.normalized();
  return m;
}

static  vtkSmartPointer<vtkTransform> toVtkMatrix(const Eigen::Affine3d& t)
{
  vtkSmartPointer<vtkTransform> vtk_t = vtkSmartPointer<vtkTransform>::New();
  vtk_t->PreMultiply();
  vtk_t->Identity();
  vtk_t->Translate(t.translation().x(),t.translation().y(), t.translation().z());
  Eigen::Vector3d rpy = t.rotation().eulerAngles(0,1,2);
  vtk_t->RotateX(vtkMath::DegreesFromRadians(rpy(0)));
  vtk_t->RotateY(vtkMath::DegreesFromRadians(rpy(1)));
  vtk_t->RotateZ(vtkMath::DegreesFromRadians(rpy(2)));
  vtk_t->Scale(1.0,1.0,1.0);
  return vtk_t;
}

double computeLength(const vtkSmartPointer<vtkPoints>& points)
{
  std::size_t num_points = points->GetNumberOfPoints();
  double total_length = 0.0;
  if(num_points < 2)
  {
    return total_length;
  }

  Eigen::Vector3d p0, pf;
  for(std::size_t i = 1; i < num_points; i++)
  {
    points->GetPoint(i-1,p0.data());
    points->GetPoint(i,pf.data());

    total_length += (pf - p0).norm();
  }
  return total_length;
}

static vtkSmartPointer<vtkPoints> applyParametricSpline(const vtkSmartPointer<vtkPoints>& points,double total_length,
                                                 double point_spacing)
{
  vtkSmartPointer<vtkPoints> new_points = vtkSmartPointer<vtkPoints>::New();
/*  new_points->DeepCopy(points);
  return new_points;*/

  // create spline
  vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
  spline->SetPoints(points);
  spline->SetParameterizeByLength(true);
  spline->ClosedOff();

  // adding first point
  Eigen::Vector3d pt_prev;
  points->GetPoint(0,pt_prev.data());
  new_points->InsertNextPoint(pt_prev.data());

  // adding remaining points by evaluating spline
  double incr = point_spacing/total_length;
  std::size_t num_points = std::ceil(total_length/point_spacing) + 1;
  double du[9];
  Eigen::Vector3d u, pt;
  bool stop_adding = false;
  for(std::size_t i = 1; i < num_points; i++)
  {
    double interv = incr * i;
    //double interv = static_cast<double>(i)/static_cast<double>(num_points);
    interv = interv > 1.0 ? 1.0 : interv ;
    if(std::abs(interv - 1.0) < EPSILON)
    {
      //break; // reach end
      stop_adding = true;
    }

    u = interv * Eigen::Vector3d::Ones();
    std::tie(u[0], u[1], u[2]) = std::make_tuple(interv, interv, interv);
    spline->Evaluate(u.data(), pt.data(), du);

    // check distance
    if((pt - pt_prev).norm() >= point_spacing)
    {
      new_points->InsertNextPoint(pt.data());
      pt_prev = pt;
    }

    if(stop_adding)
    {
      break;
    }
  }

  // add last point
  points->GetPoint(points->GetNumberOfPoints() - 1 ,pt_prev.data());
  new_points->InsertNextPoint(pt_prev.data());

  return new_points;
}

static void removeRedundant(std::vector< std::vector<vtkIdType> >& points_lists)
{
  using IdList = std::vector<vtkIdType>;
  if(points_lists.size() < 2)
  {
    return;
  }

  // sorting by count
  std::sort(points_lists.begin(), points_lists.end(),[](const IdList& l1, const IdList& l2){
    return l1.size() > l2.size();
  });

  std::vector< std::vector<vtkIdType> > new_points_lists;
  new_points_lists.push_back(points_lists.front());
  for(std::size_t i = 1; i < points_lists.size(); i++)
  {
    IdList& current_list = points_lists[i];
    IdList new_list;
    IdList all_ids;

    // create list of all ids
    for(auto& ref_list : new_points_lists)
    {
      all_ids.insert(all_ids.end(), ref_list.begin(), ref_list.end());
    }

    for(auto& id : current_list)
    {
      // add if not found in any of the previous lists
      if(std::find(all_ids.begin(), all_ids.end(),id) == all_ids.end())
      {
        new_list.push_back(id);
      }
    }

    // add if it has enough points
    if(new_list.size() > 1)
    {
      new_points_lists.push_back(new_list);
    }
  }

  points_lists.clear();
  points_lists.assign(new_points_lists.begin(), new_points_lists.end());
}

static void mergeSegments(const vtkSmartPointer<vtkPoints>& points, double merge_dist,
                          std::vector< std::vector<vtkIdType> >& points_lists)
{
  using namespace Eigen;
  using IdList = std::vector<vtkIdType>;
  if(points_lists.size() < 2)
  {
    return;
  }

  // new list of merged points
  std::vector< std::vector<vtkIdType> > new_points_lists;

  auto create_point_subset = [](const vtkSmartPointer<vtkPoints>& points,const std::vector<vtkIdType>& indices)
  {
    vtkSmartPointer<vtkPoints> points_subset = vtkSmartPointer<vtkPoints>::New();
    points_subset->DeepCopy(points);
    Vector3d p = std::numeric_limits<double>::infinity() * Vector3d::Ones();
    for(int i = 0; i < points_subset->GetNumberOfPoints(); i++)
    {
      if(std::find(indices.begin(), indices.end(), i) == indices.end())
      {
        points_subset->SetPoint(i, p.data());
      }
    }
    return points_subset;
  };

  int num_ids = std::accumulate(points_lists.begin(), points_lists.end(),0,[](int v,const IdList& l){
    return v+ l.size();
  });

  auto id_list_to_str = [](const IdList& l){
    std::stringstream ss;
    for(auto& id : l)
    {
      ss<<id <<", ";
    }
    return ss.str();
  };
  Vector3d query_point, closest_point;
  IdList merged_lists;
  IdList hide_ids;
  IdList moved_ids;
  for(std::size_t i = 0; i < points_lists.size() ; i++)
  {
    if(std::find(merged_lists.begin(), merged_lists.end(),i) != merged_lists.end())
    {
      // already merged
      continue;
    }

    IdList current_list = points_lists[i];
    if(i == points_lists.size() -1)
    {
      new_points_lists.push_back(current_list);
      break;
    }

    for(std::size_t j = i+1; j <  points_lists.size() ; j++)
    {
      vtkSmartPointer<vtkKdTree> kd_tree = vtkSmartPointer<vtkKdTree>::New();
      kd_tree->BuildLocatorFromPoints(create_point_subset(points, points_lists[j]));

      bool merge =false;
      for(auto& id : current_list)
      {
        vtkSmartPointer<vtkIdList> id_list = vtkSmartPointer<vtkIdList>::New();
        points->GetPoint(id, query_point.data());
        kd_tree->FindClosestNPoints(1,query_point.data(), id_list);
        if(id_list->GetNumberOfIds() ==0)
        {
          continue;
        }
        vtkIdType closest_id = id_list->GetId(0);

        points->GetPoint(closest_id, closest_point.data());
        double d = (query_point - closest_point).norm();
        if(merge_dist > d)
        {
          merge = true;
          break;
        }
      }

      if(merge)
      {
        current_list.insert(current_list.end(), points_lists[j].begin(), points_lists[j].end());
        merged_lists.push_back(j);
      }
    }
    new_points_lists.push_back(current_list);
  }

  points_lists.clear();
  std::copy_if(new_points_lists.begin(), new_points_lists.end(),std::back_inserter(points_lists),[](const IdList& l){
    return l.size() > 1;
  });
}

static void mergeSegmentsNew(const vtkSmartPointer<vtkPoints>& points, double merge_dist,
                             std::vector< std::vector<vtkIdType> >& points_lists)
{
  using namespace Eigen;
  using IdList = std::vector<vtkIdType>;
  if(points_lists.size() < 2)
  {
    return;
  }

  std::vector<IdList> new_points_lists;
  Vector3d query_point, closest_point;
  IdList merged_list_ids;
  IdList new_list;
  IdList moved_ids;
  for(std::size_t i = 0; i < points_lists.size() ; i++)
  {
    if(std::find(merged_list_ids.begin(), merged_list_ids.end(),i) != merged_list_ids.end())
    {
      // already merged
      continue;
    }

    IdList current_list = points_lists[i];

    for(std::size_t j = i+1; j <  points_lists.size() ; j++)
    {
      IdList next_list = points_lists[j];
      Vector3d front_current_p, back_next_p;
      points->GetPoint(current_list.front(),front_current_p.data());
      points->GetPoint(next_list.back(),back_next_p.data());

      double d = (front_current_p - back_next_p).norm();
      if(d < merge_dist)
      {
        new_list.assign(next_list.begin(), next_list.end());
        new_list.insert(new_list.end(),current_list.begin(), current_list.end());
        //merged_list_ids.push_back(i);
        merged_list_ids.push_back(j);
        CONSOLE_BRIDGE_logInform("Merged segment %lu onto %lu at front with distance %f",j, i, d);

        current_list = new_list;
        continue;
      }

      Vector3d back_current_p, front_next_p;
      points->GetPoint(current_list.back(),back_current_p.data());
      points->GetPoint(next_list.front(),front_next_p.data());
      d = (back_current_p - front_next_p).norm();
      if(d < merge_dist)
      {
        new_list.assign(current_list.begin(), current_list.end());
        new_list.insert(next_list.end(),next_list.begin(), next_list.end());
        //merged_list_ids.push_back(i);
        merged_list_ids.push_back(j);
        CONSOLE_BRIDGE_logInform("Merged segment %lu onto %lu at front from back",j, i);

        current_list = new_list;
        continue;
      }
    }
    new_points_lists.push_back(current_list);
  }
  points_lists.clear();
  std::copy_if(new_points_lists.begin(), new_points_lists.end(),std::back_inserter(points_lists),[](const IdList& l){
    return l.size() > 1;
  });

}

static vtkSmartPointer<vtkPoints> reorder(const vtkSmartPointer<vtkPoints>& points)
{
  using namespace Eigen;
  std::size_t num_points = points->GetNumberOfPoints();
  vtkSmartPointer<vtkPoints> ordered_points = vtkSmartPointer<vtkPoints>::New();
  if(num_points < 3)
  {
    ordered_points->DeepCopy(points);
    return ordered_points;
  }

  vtkSmartPointer<vtkKdTreePointLocator> kd_tree = vtkSmartPointer<vtkKdTreePointLocator>::New();
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();

  std::vector<vtkIdType> visited_indices, active_indices(num_points);
  std::iota(active_indices.begin(), active_indices.end(), 0);

  auto hide_points = [](const vtkSmartPointer<vtkPoints>& points,const std::vector<vtkIdType>& indices)
  {
    vtkSmartPointer<vtkPoints> points_subset = vtkSmartPointer<vtkPoints>::New();
    points_subset->DeepCopy(points);
    Vector3d p = std::numeric_limits<double>::infinity() * Vector3d::Ones();
    for(const auto& idx : indices)
    {
      points_subset->SetPoint(idx,p.data());
    }
    return points_subset;
  };

  vtkIdType search_point_id = active_indices.front();
  Vector3d search_point, closest_point, start_point;
  points->GetPoint(search_point_id,search_point.data());
  visited_indices.push_back(search_point_id);
  const int max_iter = num_points;
  int current_iter = 0;
  while(current_iter <= max_iter)
  {
    current_iter++;

    //std::cout<<"removing index "<<search_point_id <<", num points left: "<< active_indices.size() << std::endl;
    auto iter_pos = std::find(active_indices.begin(), active_indices.end(),search_point_id);
    if(iter_pos != active_indices.end())
    {
      active_indices.erase(std::remove(active_indices.begin(), active_indices.end(),search_point_id));
    }

    if(active_indices.empty())
    {
      break;
    }

    // set search points
    vtkSmartPointer<vtkKdTreePointLocator> kd_tree = vtkSmartPointer<vtkKdTreePointLocator>::New();
    vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
    poly_data->SetPoints(hide_points(points,visited_indices));
    kd_tree->SetDataSet(poly_data);
    kd_tree->BuildLocator();

    // find closest
    vtkIdType closest_point_id = kd_tree->FindClosestPoint(search_point.data());
    points->GetPoint(closest_point_id,closest_point.data());
    double closest_dist = (closest_point - search_point).norm();

    if(std::isinf(closest_dist))
    {
      CONSOLE_BRIDGE_logInform("infinity found");
      break;
    }

    // assigning search point for next iter
    search_point_id = closest_point_id;
    search_point = closest_point;

    if(closest_dist < EPSILON)
    {
      // numerically same point, ignore
      search_point_id = active_indices.front();
      active_indices.erase (active_indices.begin());
      points->GetPoint(search_point_id,search_point.data());
      CONSOLE_BRIDGE_logInform("Too close, skipping");
      continue;
    }

    // check if closest to start point
    points->GetPoint(visited_indices.front(),start_point.data());
    double dist_to_start = (closest_point - start_point).norm();
    if(dist_to_start < closest_dist)
    {
      // closer to start so insert at the front
      std::reverse(visited_indices.begin(), visited_indices.end());
      visited_indices.push_back(closest_point_id);
      CONSOLE_BRIDGE_logInform("Added point %i to front", closest_point_id);
    }
    else
    {
      visited_indices.push_back(closest_point_id);
      //CONSOLE_BRIDGE_logInform("Added point %i to back", closest_point_id);
    }
  }

  // copying points now
  Vector3d p;
  for(auto& id : visited_indices)
  {
    points->GetPoint(id,p.data());
    ordered_points->InsertNextPoint(p.data());
  }

  return ordered_points;
}

static std::vector<noether_msgs::ToolRasterPath> convertToPoses(const std::vector<RasterConstructData>& rasters_data)
{
  using namespace Eigen;
  std::vector<noether_msgs::ToolRasterPath> rasters_array;
  for(const RasterConstructData& rd : rasters_data)
  {
    noether_msgs::ToolRasterPath raster_path_msg;
    std::vector<PolyDataPtr> raster_segments;
    raster_segments.assign(rd.raster_segments.begin(), rd.raster_segments.end());
    if(rd.reverse_direction)
    {
      std::reverse(raster_segments.begin(), raster_segments.end());
    }

    for(const PolyDataPtr& polydata : raster_segments)
    {
      EigenSTL::vector_Isometry3d raster_poses;
      std::size_t num_points = polydata->GetNumberOfPoints();
      Vector3d p, p_next, vx, vy, vz;
      Isometry3d pose;
      for(std::size_t i = 0; i < num_points - 1; i++)
      {
        polydata->GetPoint(i,p.data());
        polydata->GetPoint(i+1,p_next.data());
        polydata->GetPointData()->GetNormals()->GetTuple(i,vz.data());
        vx = (p_next - p).normalized();
        vy = vz.cross(vx).normalized();
        vz = vx.cross(vy).normalized();
        pose = Translation3d(p) * AngleAxisd(computeRotation(vx, vy, vz));
        raster_poses.push_back(pose);
      }

      // adding last pose
      pose.translation() = p_next; // orientation stays the same as previous
      raster_poses.push_back(pose);

      if(rd.reverse_direction)
      {
        std::reverse(raster_poses.begin(), raster_poses.end());
        std::for_each(raster_poses.begin(), raster_poses.end(), [](Isometry3d& p){
          p = p * AngleAxisd(M_PI,Vector3d::UnitZ()); // rotate by 180
        });
      }

      // convert to poses msg
      geometry_msgs::PoseArray raster_poses_msg;
      std::transform(raster_poses.begin(), raster_poses.end(),
                     std::back_inserter(raster_poses_msg.poses),[](const Isometry3d& p){
        geometry_msgs::Pose pose_msg;
        tf::poseEigenToMsg(p,pose_msg);
        return pose_msg;
      });
      raster_path_msg.paths.push_back(raster_poses_msg);

    }
    rasters_array.push_back(raster_path_msg);
  }

  return rasters_array;
}

namespace tool_path_planner
{
PlaneSlicerRasterGenerator::PlaneSlicerRasterGenerator()
{

}

PlaneSlicerRasterGenerator::~PlaneSlicerRasterGenerator()
{

}

void PlaneSlicerRasterGenerator::setInput(pcl::PolygonMesh::ConstPtr mesh)
{
  mesh_data_ = vtkSmartPointer<vtkPolyData>::New();
  pcl::VTKUtils::mesh2vtk(*mesh, mesh_data_);
  mesh_data_->BuildLinks();
  mesh_data_->BuildCells();

  if(mesh_data_->GetPointData()->GetNormals() && mesh_data_->GetCellData()->GetNormals() )
  {
    return; // normal data is available
  }

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

void PlaneSlicerRasterGenerator::setInput(const shape_msgs::Mesh& mesh)
{
  pcl::PolygonMesh::Ptr pcl_mesh = boost::make_shared<pcl::PolygonMesh>();
  noether_conversions::convertToPCLMesh(mesh,*pcl_mesh);
  setInput(pcl_mesh);
}

boost::optional<std::vector<noether_msgs::ToolRasterPath> >
PlaneSlicerRasterGenerator::generate(const PlaneSlicerRasterGenerator::Config& config)
{
  using namespace Eigen;
  using IDVec = std::vector<vtkIdType>;

  boost::optional<std::vector<noether_msgs::ToolRasterPath> > rasters = boost::none;
  if(!mesh_data_)
  {
    CONSOLE_BRIDGE_logInform("%s No mesh data has been provided",getName().c_str());
  }
  // computing mayor axis using oob
  vtkSmartPointer<vtkOBBTree> oob = vtkSmartPointer<vtkOBBTree>::New();
  Vector3d corner, x_dir, y_dir, z_dir, sizes;// x is longest, y is mid and z is smallest
  oob->ComputeOBB(mesh_data_,corner.data(), x_dir.data(), y_dir.data(), z_dir.data(), sizes.data());

  // Compute the center of mass
  Vector3d origin;
  vtkSmartPointer<vtkCenterOfMass> cog_filter = vtkSmartPointer<vtkCenterOfMass>::New();
  cog_filter->SetInputData(mesh_data_);
  cog_filter->SetUseScalarsAsWeights(false);
  cog_filter->Update();
  cog_filter->GetCenter(origin.data());

  // computing transformation matrix
  Affine3d t = Translation3d(origin) * AngleAxisd(computeRotation(x_dir, y_dir, z_dir));
  Affine3d t_inv = t.inverse();

  // transforming data
  vtkSmartPointer<vtkTransform> vtk_transform = toVtkMatrix(t_inv);

  vtkSmartPointer<vtkTransformFilter> transform_filter = vtkSmartPointer<vtkTransformFilter>::New();
  transform_filter->SetInputData(mesh_data_);
  transform_filter->SetTransform(vtk_transform);
  transform_filter->TransformAllInputVectorsOn();
  transform_filter->Update();
  vtkSmartPointer<vtkPolyData> transformed_mesh_data = transform_filter->GetPolyDataOutput();

  // compute bounds
  VectorXd bounds(6), abs_bounds(6);
  transformed_mesh_data->GetBounds(bounds.data());
  abs_bounds = bounds.cwiseAbs();

  // calculating size
  sizes.x() = abs_bounds[0] > abs_bounds[1] ? 2.0 * abs_bounds[0] : 2.0 *abs_bounds[1];
  sizes.y() = abs_bounds[2] > abs_bounds[3] ? 2.0 * abs_bounds[2] : 2.0 *abs_bounds[3];
  sizes.z() = abs_bounds[4] > abs_bounds[5] ? 2.0 * abs_bounds[4] : 2.0 *abs_bounds[5];

  // now cutting the mesh with planes along the y axis
  std::size_t num_planes = std::ceil(sizes[1]/config.raster_spacing);
  Isometry3d rotation_offset = Isometry3d::Identity() * AngleAxisd(config.raster_rot_offset, Vector3d::UnitZ());
  Vector3d raster_dir = rotation_offset * Vector3d::UnitY();
  Vector3d start_loc = -config.raster_spacing * std::ceil(num_planes * 0.5)* raster_dir + Vector3d::Zero();

  vtkSmartPointer<vtkAppendPolyData> raster_data = vtkSmartPointer<vtkAppendPolyData>::New();
  for(int i = 0; i < num_planes + 1 ; i++)
  {
    vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
    vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
    vtkSmartPointer<vtkStripper> stripper = vtkSmartPointer<vtkStripper>::New();

    Vector3d current_loc = start_loc + i * config.raster_spacing * raster_dir;
    plane->SetOrigin(current_loc.x(), current_loc.y(), current_loc.z());
    plane->SetNormal(raster_dir.x() ,raster_dir.y(), raster_dir.z());

    cutter->SetCutFunction(plane);
    cutter->SetInputData(transformed_mesh_data);
    cutter->Update();

    stripper->SetInputConnection(cutter->GetOutputPort());
    stripper->JoinContiguousSegmentsOff();
    stripper->SetMaximumLength(mesh_data_->GetNumberOfPoints());
    stripper->Update();

    if(stripper->GetErrorCode() != vtkErrorCode::NoError)
    {
      continue;
    }

    for(int r = 0; r < stripper->GetNumberOfOutputPorts(); r++)
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
  std::size_t num_slices = raster_data->GetTotalNumberOfInputConnections();
  std::vector<RasterConstructData> rasters_data_vec;
  bool reverse = true;
  for(std::size_t i = 0; i < num_slices; i++)
  {
    RasterConstructData r;
    reverse = !reverse;
    r.reverse_direction = reverse;

    // collecting raster segments based on min hole size
    vtkSmartPointer<vtkPolyData> raster_lines = raster_data->GetInput(i);
    vtkIdType* indices;
    vtkIdType num_points;
    vtkIdType num_lines = raster_lines->GetNumberOfLines();
    vtkPoints* points = raster_lines->GetPoints();
    vtkCellArray* cells = raster_lines->GetLines();

    std::vector<IDVec> raster_ids;
    CONSOLE_BRIDGE_logInform("%s raster %i has %i lines and %i points",getName().c_str(), i,
                             raster_lines->GetNumberOfLines(), raster_lines->GetNumberOfPoints());

    if(num_lines ==0)
    {
      continue;
    }

    unsigned int lineCount = 0;
    for (cells->InitTraversal(); cells->GetNextCell(num_points, indices);
         lineCount++)
    {
      IDVec point_ids;
      for (vtkIdType i = 0; i < num_points; i++)
      {
        if(std::find(point_ids.begin(), point_ids.end(), indices[i]) != point_ids.end())
        {
          continue;
        }
        point_ids.push_back(indices[i]);
      }
      if(point_ids.empty())
      {
        continue;
      }

      // removing duplicates
      auto iter = std::unique(point_ids.begin(), point_ids.end());
      point_ids.erase(iter,point_ids.end());

      // adding
      raster_ids.push_back(point_ids);
    }

    // remove redundant indices
    removeRedundant(raster_ids);

    // merging segments
    mergeSegmentsNew(raster_lines->GetPoints(),config.merge_threshold,raster_ids);

    for(auto& rpoint_ids: raster_ids)
    {
      // Populating with points
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
      std::for_each(rpoint_ids.begin(), rpoint_ids.end(),[&points, &raster_lines](vtkIdType& id){
        std::array<double,3> p;
        raster_lines->GetPoint(id, p.data());
        points->InsertNextPoint(p.data());
      });

      // compute length and add points if segment length is greater than threshold
      double line_length = computeLength(points);
      if(line_length > config.min_segment_size && points->GetNumberOfPoints()> 1)
      {
        // order points
        //decltype(points) new_points = reorder(points);

        // enforce point spacing
        //new_points = applyParametricSpline(points,line_length,config.point_spacing);
        decltype(points) new_points = applyParametricSpline(points,line_length,config.point_spacing);

        // add points to segment now
        PolyDataPtr segment_data = PolyDataPtr::New();
        segment_data->SetPoints(new_points);

        // transforming to original coordinate system
        transform_filter = vtkSmartPointer<vtkTransformFilter>::New();
        transform_filter->SetInputData(segment_data);
        transform_filter->SetTransform(toVtkMatrix(t));
        transform_filter->TransformAllInputVectorsOn();
        transform_filter->Update();
        segment_data = transform_filter->GetPolyDataOutput();

        // inserting normals
        if(!insertNormals(config.search_radius,segment_data))
        {
          CONSOLE_BRIDGE_logError("%s failed to insert normals to segment %lu of raster %lu",
                                  getName().c_str(), r.raster_segments.size(),
                                  i);
          return boost::none;
        }

        // saving into raster
        r.raster_segments.push_back(segment_data);
        r.segment_lengths.push_back(line_length);
      }
    }

    rasters_data_vec.push_back(r);
  }

  // converting to poses msg now
  rasters = convertToPoses(rasters_data_vec);
  return rasters;
}

boost::optional<std::vector<noether_msgs::ToolRasterPath> >
PlaneSlicerRasterGenerator::generate(const shape_msgs::Mesh& mesh, const PlaneSlicerRasterGenerator::Config& config)
{
  setInput(mesh);
  return generate(config);
}

boost::optional<std::vector<noether_msgs::ToolRasterPath> >
PlaneSlicerRasterGenerator::generate(pcl::PolygonMesh::ConstPtr mesh, const PlaneSlicerRasterGenerator::Config& config)
{
  setInput(mesh);
  return generate(config);
}

bool PlaneSlicerRasterGenerator::insertNormals(const double search_radius,
                   vtkSmartPointer<vtkPolyData>& data)
{
  // Find closest cell to each point and uses its normal vector
  vtkSmartPointer<vtkDoubleArray> new_normals = vtkSmartPointer<vtkDoubleArray>::New();
  new_normals->SetNumberOfComponents(3);
  new_normals->SetNumberOfTuples(data->GetPoints()->GetNumberOfPoints());

  // get normal data
  vtkSmartPointer<vtkDataArray> normal_data = mesh_data_->GetPointData()->GetNormals();

  if(!normal_data)
  {
    CONSOLE_BRIDGE_logError("%s Normal data is not available", getName().c_str());
    return false;
  }

  Eigen::Vector3d normal_vect = Eigen::Vector3d::UnitZ();
  for(int i = 0; i < data->GetPoints()->GetNumberOfPoints(); ++i)
  {
    // locate closest cell
    Eigen::Vector3d query_point;
    vtkSmartPointer<vtkIdList> id_list = vtkSmartPointer<vtkIdList>::New();
    data->GetPoints()->GetPoint(i, query_point.data());
    kd_tree_->FindPointsWithinRadius(search_radius,query_point.data(),id_list);
    if(id_list->GetNumberOfIds() < 1)
    {
      CONSOLE_BRIDGE_logWarn("%s FindPointsWithinRadius found no points for normal averaging, using closests",
                             getName().c_str());
      kd_tree_->FindClosestNPoints(1, query_point.data(),id_list);

      if(id_list->GetNumberOfIds() < 1)
      {
        CONSOLE_BRIDGE_logError("%s failed to find closest for normal computation",getName().c_str());
        return false;
      }
    }

    // compute normal average
    normal_vect = Eigen::Vector3d::Zero();
    std::size_t num_normals = 0;
    for(std::size_t p = 0; p < id_list->GetNumberOfIds(); p++)
    {
      Eigen::Vector3d temp_normal, query_point, closest_point;
      vtkIdType p_id = id_list->GetId(p);
      vtkIdType cell_id;
      int sub_index;
      double dist;

      if(p_id < 0)
      {
        CONSOLE_BRIDGE_logError("%s point id is invalid",getName().c_str());
        continue;
      }

      // get normal and add it to average
      normal_data->GetTuple(p_id,temp_normal.data());
      normal_vect += temp_normal.normalized();
      num_normals++;
    }

    normal_vect /= num_normals;
    normal_vect.normalize();

    // save normal
    new_normals->SetTuple3(i,normal_vect(0),normal_vect(1),normal_vect(2));
  }
  data->GetPointData()->SetNormals(new_normals);
  return true;
}

std::string PlaneSlicerRasterGenerator::getName()
{
  return getClassName<decltype(*this)>();
}

} /* namespace tool_path_planner */
