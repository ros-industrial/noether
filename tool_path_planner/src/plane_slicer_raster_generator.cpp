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
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkCutter.h>
#include <vtkPlane.h>
#include <vtkAppendPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkCenterOfMass.h>
#include <vtkParametricSpline.h>
#include <vtkErrorCode.h>

#include <boost/make_shared.hpp>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <console_bridge/console.h>
#include <noether_conversions/noether_conversions.h>
#include "tool_path_planner/utilities.h"
#include "tool_path_planner/plane_slicer_raster_generator.h"

static const double EPSILON = 1e-8;

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

  // create spline
  vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
  spline->SetPoints(points);
  spline->ParameterizeByLengthOn();
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
  for(std::size_t i = 1; i < num_points; i++)
  {
    double interv = incr * i;
    interv = interv > 1.0 ? 1.0 : interv ;
    if(std::abs(interv - 1.0) < EPSILON)
    {
      break; // reach end
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
  }

  // add last point
  points->GetPoint(points->GetNumberOfPoints() - 1 ,pt_prev.data());
  new_points->InsertNextPoint(pt_prev.data());

  return new_points;
}

static std::vector<geometry_msgs::PoseArray> convertToPoses(const std::vector<RasterConstructData>& rasters_data)
{
  using namespace Eigen;
  std::vector<geometry_msgs::PoseArray> rasters_poses_msg;
  for(const RasterConstructData& rd : rasters_data)
  {
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

      rasters_poses_msg.push_back(raster_poses_msg);
    }
  }

  return rasters_poses_msg;
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
}

void PlaneSlicerRasterGenerator::setInput(const shape_msgs::Mesh& mesh)
{
  pcl::PolygonMesh::Ptr pcl_mesh = boost::make_shared<pcl::PolygonMesh>();
  noether_conversions::convertToPCLMesh(mesh,*pcl_mesh);
  setInput(pcl_mesh);
}

boost::optional<std::vector<geometry_msgs::PoseArray> >
PlaneSlicerRasterGenerator::generate(const PlaneSlicerRasterGenerator::Config& config)
{
  using namespace Eigen;
  boost::optional<std::vector<geometry_msgs::PoseArray> > rasters = boost::none;
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

    Vector3d current_loc = start_loc + i * config.raster_spacing * raster_dir;
    plane->SetOrigin(current_loc.x(), current_loc.y(), current_loc.z());
    plane->SetNormal(raster_dir.x() ,raster_dir.y(), raster_dir.z());

    cutter->SetCutFunction(plane);
    cutter->SetInputData(transformed_mesh_data);
    cutter->Update();

    if(cutter->GetErrorCode() != vtkErrorCode::NoError)
    {
      continue;
    }

    raster_data->AddInputData(cutter->GetOutput());
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
    double line_gap = -1.0;
    std::vector<vtkIdType> point_ids;
    for(std::size_t l = 0; l < raster_lines->GetNumberOfLines(); l++)
    {
      // check current line
      vtkSmartPointer<vtkIdList> id_list = vtkSmartPointer<vtkIdList>::New();
      raster_lines->GetLines()->GetCell(l,id_list);
      std::size_t num_points = id_list->GetNumberOfIds();
      if(num_points == 0)
      {
        line_gap = 0.0;
        continue; // do nothing
      }

      // computing line gap
      if(!point_ids.empty())
      {
        Eigen::Vector3d prev_point, first_point;
        raster_lines->GetPoint(point_ids.back(), prev_point.data());
        raster_lines->GetPoint(id_list->GetId(0), first_point.data());
        line_gap = (first_point - prev_point).norm();
      }

      // create new segment if line breaks due to hole
      if(line_gap > config.min_hole_size)
      {
        // Populating with points
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        std::for_each(point_ids.begin(), point_ids.end(),[&points, &raster_lines](decltype(point_ids)::value_type& id){
          std::array<double,3> p;
          raster_lines->GetPoint(id, p.data());
          points->InsertNextPoint(p.data());
        });

        // compute length and add points if segment length is greater than threshold
        double l = computeLength(points);
        if(l > config.min_segment_size)
        {
          // enforce point spacing
          decltype(points) new_points = applyParametricSpline(points,l,config.point_spacing);

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
          r.segment_lengths.push_back(l);
        }

        // clear data
        point_ids.clear();
      }

      // adding points ids in current line to segment
      for(std::size_t c = 0; c < num_points; c++)
      {
        vtkIdType point_id = id_list->GetId(c);
        if(std::find(point_ids.begin(), point_ids.end(), point_id) == point_ids.end())
        {
          point_ids.push_back(point_id);
        }
      }
    }
    rasters_data_vec.push_back(r);
  }

  // converting to poses msg now
  rasters = convertToPoses(rasters_data_vec);
  return rasters;
}

boost::optional<std::vector<geometry_msgs::PoseArray> >
PlaneSlicerRasterGenerator::generate(const shape_msgs::Mesh& mesh, const PlaneSlicerRasterGenerator::Config& config)
{
  setInput(mesh);
  return generate(config);
}

boost::optional<std::vector<geometry_msgs::PoseArray> >
PlaneSlicerRasterGenerator::generate(pcl::PolygonMesh::ConstPtr mesh, const PlaneSlicerRasterGenerator::Config& config)
{
  setInput(mesh);
  return generate(config);
}

bool PlaneSlicerRasterGenerator::insertNormals(const double search_radius,
                   vtkSmartPointer<vtkPolyData>& data)
{
  // Find closest cell to each point and uses its normal vector
  vtkSmartPointer<vtkDoubleArray> new_norm = vtkSmartPointer<vtkDoubleArray>::New();
  new_norm->SetNumberOfComponents(3);
  new_norm->SetNumberOfTuples(data->GetPoints()->GetNumberOfPoints());

  for(int i = 0; i < data->GetPoints()->GetNumberOfPoints(); ++i)
  {
    // locate closest cell
    std::array<double,3> query_point;
    vtkSmartPointer<vtkIdList> id_list = vtkSmartPointer<vtkIdList>::New();
    data->GetPoints()->GetPoint(i, query_point.data());
    kd_tree_->FindPointsWithinRadius(search_radius,query_point.data(),id_list);
    if(id_list->GetNumberOfIds() < 1)
    {
      CONSOLE_BRIDGE_logError("FindPointsWithinRadius returned no closest points");
      return false;
    }

    // compute normal average
    Eigen::Vector3d normal_vect = Eigen::Vector3d::Zero();
    for(std::size_t p = 0; p < id_list->GetNumberOfIds(); p++)
    {
      Eigen::Vector3d temp_normal, query_point, closest_point;
      vtkIdType p_id = id_list->GetId(p);
      vtkIdType cell_id;
      int sub_index;
      double dist;

      // find closes point cell id first
      mesh_data_->GetPoints()->GetPoint(p_id, query_point.data());
      cell_locator_->FindClosestPoint(query_point.data(),closest_point.data(),cell_id,sub_index,dist);

      // get normal and add it to average
      mesh_data_->GetCellData()->GetNormals()->GetTuple(cell_id,temp_normal.data());
      normal_vect += temp_normal.normalized();
    }

    normal_vect /= id_list->GetNumberOfIds();
    normal_vect.normalize();

    // get normal
    new_norm->SetTuple3(i,normal_vect(0),normal_vect(1),normal_vect(2));
  }
  data->GetPointData()->SetNormals(new_norm);
  return true;
}

std::string PlaneSlicerRasterGenerator::getName()
{
  return getClassName<decltype(*this)>();
}

} /* namespace tool_path_planner */
