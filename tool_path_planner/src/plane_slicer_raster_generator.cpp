/*
 * plane_slicer_raster_generator.cpp
 *
 *  Created on: Dec 26, 2019
 *      Author: jrgnicho
 */

#include <vtkMath.h>
#include <vtkBoundingBox.h>
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
#include <vtkErrorCode.h>

#include <boost/make_shared.hpp>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <console_bridge/console.h>
#include <noether_conversions/noether_conversions.h>
#include "tool_path_planner/utilities.h"
#include "tool_path_planner/plane_slicer_raster_generator.h"

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

  return rasters;
}

boost::optional<std::vector<geometry_msgs::PoseArray> >
PlaneSlicerRasterGenerator::generate(const shape_msgs::Mesh& mesh, const PlaneSlicerRasterGenerator::Config& config)
{
}

boost::optional<std::vector<geometry_msgs::PoseArray> >
PlaneSlicerRasterGenerator::generate(pcl::PolygonMesh::ConstPtr mesh, const PlaneSlicerRasterGenerator::Config& config)
{
}

std::string PlaneSlicerRasterGenerator::getName()
{
  return getClassName<decltype(*this)>();
}

} /* namespace tool_path_planner */
