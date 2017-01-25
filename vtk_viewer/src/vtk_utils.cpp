/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include "vtk_viewer/vtk_utils.h"

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>  // MESH conversion utils VTK<->PCL
#include <pcl/surface/mls.h>

#include <pcl/surface/grid_projection.h> // surface reconstruction by Rosie Li based on:
// Polygonizing Extremal Surfaces with Manifold Guarantees, Ruosi Li, et. al.

#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkMath.h>

#include <vtkTransform.h>
#include <vtkVersion.h>
#include <vtkProperty.h>
#include <vtkDelaunay2D.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSTLReader.h>

#include <vtkPolyDataNormals.h>
#include <vtkDoubleArray.h>
#include <vtkMarchingCubes.h>
#include <vtkCurvatures.h>

#include <vtkSurfaceReconstructionFilter.h>
#include <vtkReverseSense.h>
#include <vtkContourFilter.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkKdTreePointLocator.h>
#include <vtkTriangle.h>
#include <vtkCleanPolyData.h>

#include <vtkImplicitSelectionLoop.h>
#include <vtkClipPolyData.h>


namespace vtk_viewer
{


vtkSmartPointer<vtkPoints> createPlane()
{
  // Create points on an XY grid with a sinusoidal Z component
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  unsigned int gridSize = 10;
  for(unsigned int x = 0; x < gridSize; x++)
    {
    for(unsigned int y = 0; y < gridSize; y++)
      {
        points->InsertNextPoint(x  , y , 1.0 * cos(double(x)/2.0) - 1.0 * sin(double(y)/2.0) + vtkMath::Random(0.0, 0.001));
      }
    }

  // Add the grid points to a polydata object
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  return points;
}

vtkSmartPointer<vtkPolyData> createMesh(vtkSmartPointer<vtkPoints> points , double sample_spacing, double neigborhood_size)
{
  if(!points)
  {
    points = createPlane();
  }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  // surface reconstruction
  vtkSmartPointer<vtkContourFilter> cf;
  vtkSmartPointer<vtkSurfaceReconstructionFilter> surf =
  vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();

  surf->SetInputData(polydata);
  surf->SetSampleSpacing(sample_spacing);
  surf->SetNeighborhoodSize(neigborhood_size);
  surf->Update();


  cf = vtkSmartPointer<vtkContourFilter>::New();
  cf->SetInputConnection(surf->GetOutputPort());
  cf->SetValue(0, 0.0);
  cf->Update();

  // Sometimes the contouring algorithm can create a volume whose gradient
  // vector and ordering of polygon (using the right hand rule) are
  // inconsistent. vtkReverseSense cures this problem.
  vtkSmartPointer<vtkReverseSense> reverse = vtkSmartPointer<vtkReverseSense>::New();
  reverse->SetInputConnection(cf->GetOutputPort());
  reverse->ReverseCellsOn();
  reverse->ReverseNormalsOn();
  reverse->Update();

  return reverse->GetOutput();
}

void cleanMesh(const vtkSmartPointer<vtkPoints>& points, vtkSmartPointer<vtkPolyData>& mesh)
{
  // create KD tree object for searching for points
  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  data->SetPoints(points);
  vtkSmartPointer<vtkKdTreePointLocator> kd_tree = vtkSmartPointer<vtkKdTreePointLocator>::New();
  kd_tree->SetDataSet(data);
  kd_tree->BuildLocator();

  vtkSmartPointer<vtkCellArray> cells = mesh->GetPolys();
  int size = cells->GetNumberOfCells();
  // loop through all cells, mark cells for deletion if they do not have enough points nearby
  for(int i = 0; i < size; ++i)
  {
    // get cell centroid and size
    vtkCell* cell = mesh->GetCell(i);
    if(cell)
    {
      // get cell centroid and max leg length
      vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
      double p0[3];
      double p1[3];
      double p2[3];
      double center[3];
      triangle->GetPoints()->GetPoint(0, p0);
      triangle->GetPoints()->GetPoint(1, p1);
      triangle->GetPoints()->GetPoint(2, p2);

      double a = pt_dist(&p0[0], &p1[0]);
      double b = pt_dist(&p1[0], &p2[0]);
      double c = pt_dist(&p0[0], &p2[0]);
      triangle->TriangleCenter(p0, p1, p2, center);

      // search for points in radius around the cell centroid
      double max = (a > b ? a : b);
      max = max > c ? max : c;
      double radius =  2.0 * sqrt(max);
      vtkSmartPointer<vtkIdList> pts = vtkSmartPointer<vtkIdList>::New();
      kd_tree->FindPointsWithinRadius(radius, center, pts);

      // if number of desired points not found, mark cell for deletion
      if(pts->GetNumberOfIds() < 6)
      {
        mesh->DeleteCell(i);
      }
    }
  }

  // delete all marked cells
  mesh->RemoveDeletedCells();

  // clean mesh: merge points, remove unused points, and remove
  // degenerate cells (cells where two points are very close to one another)
  vtkSmartPointer<vtkCleanPolyData> clean_polydata = vtkSmartPointer<vtkCleanPolyData>::New();
  clean_polydata->SetInputData(mesh);
  clean_polydata->Update();

  mesh = clean_polydata->GetOutput();

}

void visualizePlane(vtkSmartPointer<vtkPolyData> &polydata)
{
  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> triangulated_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  triangulated_mapper->SetInputData(polydata);

  vtkSmartPointer<vtkActor> triangulated_actor = vtkSmartPointer<vtkActor>::New();
  triangulated_actor->SetMapper(triangulated_mapper);

  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> render_window = vtkSmartPointer<vtkRenderWindow>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> render_interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

  render_window->AddRenderer(renderer);
  render_interactor->SetRenderWindow(render_window);

  // Add the actor to the scene
  renderer->AddActor(triangulated_actor);
  renderer->SetBackground(.3, .6, .3); // Background color green

  // Render and interact
  render_window->Render();
  render_interactor->Start();
}

vtkSmartPointer<vtkPolyData> readSTLFile(std::string file)
{
  vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
  reader->SetFileName(file.c_str());
  reader->SetMerging(1);
  reader->Update();

  return reader->GetOutput();
}

bool loadPCDFile(std::string file, vtkSmartPointer<vtkPolyData>& polydata, std::string background, bool return_mesh)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file.c_str(), *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file \n");
      return false;
    }

  if(background != "")
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr bg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (background.c_str(), *bg_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file \n");
    }
    else
    {
      // perform background subtraction
      removeBackground(*cloud, *bg_cloud);
    }
  }

  // use PCL to mesh the point cloud
  pclGridProjectionMesh(cloud, polydata);

  //vtkSurfaceReconstructionMesh(cloud, polydata);

  return true;
}

void vtkSurfaceReconstructionMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vtkSmartPointer<vtkPolyData> &mesh)
{
  // use MLS filter to smooth data and calculate normals (TODO: Normal data may not be oriented correctly)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
  pcl::PointCloud<pcl::PointXYZ> mls_points;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Set parameters
  mls.setComputeNormals (false);
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.01);

  // Reconstruct
  mls.process(mls_points);

  if(!mesh)
  {
    mesh = vtkSmartPointer<vtkPolyData>::New();
  }

  //Convert pcl::PointCloud<T> -> vtkPolyData
  vtkSmartPointer<vtkPolyData> point_data = vtkSmartPointer<vtkPolyData>::New();
  //const pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>(mls_points));

  PCLtoVTK(mls_points, point_data);

  // Mesh
  mesh = createMesh(point_data->GetPoints(), 0.01, 20);
  cleanMesh(point_data->GetPoints(), mesh);

}

void pclGridProjectionMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vtkSmartPointer<vtkPolyData>& mesh)
{
  // use MLS filter to smooth data and calculate normals (TODO: Normal data may not be oriented correctly)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  pcl::PointCloud<pcl::PointNormal> mls_points;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Set parameters
  mls.setComputeNormals (true);
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.01);

  // Reconstruct
  mls.process(mls_points);

  if(!mesh)
  {
    mesh = vtkSmartPointer<vtkPolyData>::New();
  }

  // Perform Grid Projection point cloud meshing (Polygonizing Extremal Surfaces with Manifold Guarantees, Ruosi Li, et. al.)
  pcl::GridProjection<pcl::PointNormal> grid_surf;
  pcl::PolygonMesh output_mesh;
  const pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>(mls_points));

  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_with_normals);

  // Set parameters
  grid_surf.setResolution(0.003);  // this parameter is the main one which affects the smoothness of the mesh
  grid_surf.setPaddingSize(1);
  grid_surf.setMaxBinarySearchLevel(6); // default is 10
  grid_surf.setNearestNeighborNum(20); // default is 50
  grid_surf.setSearchMethod(tree2);

  // Mesh
  grid_surf.setInputCloud(cloud_with_normals);
  grid_surf.reconstruct(output_mesh);

  // Point cloud of output mesh does not have normals, compute normals first before returning
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

  // Convert pcl::PCLPointCloud2 -> pcl::PointCloud<T>
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(output_mesh.cloud, *temp_cloud);
  ne.setInputCloud (temp_cloud);

  ne.setViewPoint(0,0,5.0);
  ne.setRadiusSearch (0.01);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

  // Compute normals and add to original points
  ne.compute (*cloud_normals2);
  pcl::PointCloud<pcl::PointNormal>::Ptr new_cloud(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*temp_cloud, *cloud_normals2, *new_cloud);

  // Convert pcl::PointCloud<PointNormal> -> pcl::PCLPointCloud2 and store in original mesh object
  pcl::toPCLPointCloud2(*new_cloud, output_mesh.cloud);

  // Convert mesh object to VTK
  pcl::VTKUtils::convertToVTK(output_mesh, mesh);

}

pcl::PolygonMesh pclGridProjectionMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // use MLS filter to smooth data and calculate normals (TODO: Normal data may not be oriented correctly)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  pcl::PointCloud<pcl::PointNormal> mls_points;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Set parameters
  mls.setComputeNormals (true);
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.01);

  // Reconstruct
  mls.process(mls_points);

  // Perform Grid Projection point cloud meshing (Polygonizing Extremal Surfaces with Manifold Guarantees, Ruosi Li, et. al.)
  pcl::GridProjection<pcl::PointNormal> grid_surf;
  pcl::PolygonMesh output_mesh;
  const pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>(mls_points));

  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_with_normals);

  // Set parameters
  grid_surf.setResolution(0.003);  // this parameter is the main one which affects the smoothness of the mesh
  grid_surf.setPaddingSize(1);
  grid_surf.setMaxBinarySearchLevel(6); // default is 10
  grid_surf.setNearestNeighborNum(20); // default is 50
  grid_surf.setSearchMethod(tree2);

  // Mesh
  grid_surf.setInputCloud(cloud_with_normals);
  grid_surf.reconstruct(output_mesh);

  // Point cloud of output mesh does not have normals, compute normals first before returning
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

  // Convert pcl::PCLPointCloud2 -> pcl::PointCloud<T>
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(output_mesh.cloud, *temp_cloud);
  ne.setInputCloud (temp_cloud);

  ne.setViewPoint(0,0,5.0);
  ne.setRadiusSearch (0.01);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

  // Compute normals and add to original points
  ne.compute (*cloud_normals2);
  pcl::PointCloud<pcl::PointNormal>::Ptr new_cloud(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*temp_cloud, *cloud_normals2, *new_cloud);

  // Convert pcl::PointCloud<PointNormal> -> pcl::PCLPointCloud2 and store in original mesh object
  pcl::toPCLPointCloud2(*new_cloud, output_mesh.cloud);

  return output_mesh;
}

void removeBackground(pcl::PointCloud<pcl::PointXYZ>& cloud, const pcl::PointCloud<pcl::PointXYZ>& background)
{
  if(cloud.points.size() != background.points.size())
  {
    return;
  }

  for (int i = 0; i < cloud.points.size(); ++i)
  {
    if(fabs(cloud.points[i].z - background.points[i].z) < 0.05 || std::isnan(background.points[i].z) || cloud.points[i].y > 0.75)
    {
      cloud.points[i].x = NAN;
      cloud.points[i].y = NAN;
      cloud.points[i].z = NAN;
    }
  }

  // create new point cloud with NaN's removed
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  for (int i = 0; i < cloud.points.size(); ++i)
  {
    if(!std::isnan(cloud.points[i].x))
    {
      pcl::PointXYZ pt;
      pt.x = cloud.points[i].x;
      pt.y = cloud.points[i].y;
      pt.z = cloud.points[i].z;
      cloud2.push_back(pt);
    }
  }
  cloud = cloud2;
}

void PCLtoVTK(const pcl::PointCloud<pcl::PointXYZ> &cloud, vtkPolyData* const pdata)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  for (int i = 0; i < cloud.points.size(); ++i)
  {
    if(std::isnan(cloud.points[i].x) || std::isnan(cloud.points[i].y) || std::isnan(cloud.points[i].z))
    {
      continue;
    }
    const double pt[3] = {double(cloud.points[i].x), double(cloud.points[i].y), double(cloud.points[i].z)};
    points->InsertNextPoint(pt);
  }

  // Create a temporary PolyData and add the points to it
  vtkSmartPointer<vtkPolyData> tempPolyData = vtkSmartPointer<vtkPolyData>::New();
  tempPolyData->SetPoints(points);

  pdata->DeepCopy(tempPolyData);
}

void VTKtoPCL(vtkPolyData* const pdata, pcl::PointCloud<pcl::PointNormal> &cloud)
{
  for (int i = 0; i < pdata->GetPoints()->GetNumberOfPoints(); ++i)
  {
    pcl::PointNormal pt;
    double* ptr = pdata->GetPoints()->GetPoint(i);
    pt.x = ptr[0];
    pt.y = ptr[1];
    pt.z = ptr[2];

    double* norm = pdata->GetPointData()->GetNormals()->GetTuple(i);
    pt.normal_x = norm[0];
    pt.normal_y = norm[1];
    pt.normal_z = norm[2];

    cloud.push_back(pt);
  }

}

vtkSmartPointer<vtkPolyData> estimateCurvature(vtkSmartPointer<vtkPolyData> mesh, int method)
{
  vtkSmartPointer<vtkCurvatures> curvature_filter = vtkSmartPointer<vtkCurvatures>::New();
    curvature_filter->SetInputData(mesh);
    switch(method)
    {
    case 0:
      curvature_filter->SetCurvatureTypeToMinimum();
      break;
    case 1:
      curvature_filter->SetCurvatureTypeToMaximum();
      break;
    case 2:
      curvature_filter->SetCurvatureTypeToGaussian();
      break;
    default:
      curvature_filter->SetCurvatureTypeToMean();
      break;
    }
    curvature_filter->Update();

    return curvature_filter->GetOutput();
}

void generateNormals(vtkSmartPointer<vtkPolyData>& data, int flip_normals)
{
  vtkSmartPointer<vtkPolyDataNormals> normal_generator = vtkSmartPointer<vtkPolyDataNormals>::New();
  normal_generator->SetInputData(data);
  normal_generator->ComputePointNormalsOn();
  normal_generator->ComputeCellNormalsOn();

  // Optional settings
  normal_generator->SetFeatureAngle(0.5);
  normal_generator->SetSplitting(0);
  normal_generator->SetConsistency(1);
  normal_generator->SetAutoOrientNormals(flip_normals);
  if(!data->GetPointData()->GetNormals())
  {
    normal_generator->SetComputePointNormals(1);
  }
  else
  {
    normal_generator->SetComputePointNormals(0);
  }
  if(!data->GetCellData()->GetNormals())
  {
    normal_generator->SetComputeCellNormals(1);
  }
  else
  {
    normal_generator->SetComputeCellNormals(0);
  }
  normal_generator->SetFlipNormals(0);
  normal_generator->SetNonManifoldTraversal(0);

//  vtkSmartPointer<vtkCleanPolyData> clean_polydata = vtkSmartPointer<vtkCleanPolyData>::New();
//  clean_polydata->SetInputData(mesh);
//  clean_polydata->Update();

  normal_generator->Update();

  vtkDataArray* normals = normal_generator->GetOutput()->GetPointData()->GetNormals();
  if(normals)
  {
    data->GetPointData()->SetNormals(normals);
  }

  vtkDataArray* normals2 = normal_generator->GetOutput()->GetCellData()->GetNormals();
  if(normals2)
  {
    data->GetCellData()->SetNormals(normals2);
  }
}

vtkSmartPointer<vtkPolyData> upsampleMesh(vtkSmartPointer<vtkPolyData> mesh, double distance)
{
  vtkSmartPointer<vtkPolyDataPointSampler> point_sampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
  point_sampler->SetDistance(distance);
  point_sampler->SetInputData(mesh);
  point_sampler->Update();

  // Resize the mesh and insert the new points
  int old_size = mesh->GetPoints()->GetNumberOfPoints();
  int new_size = point_sampler->GetOutput()->GetPoints()->GetNumberOfPoints();

  vtkSmartPointer<vtkPolyData> updated_mesh = vtkSmartPointer<vtkPolyData>::New();
  updated_mesh->SetPoints(mesh->GetPoints());
  int size = old_size + new_size;
  updated_mesh->GetPoints()->Resize(size);

  updated_mesh->GetPoints()->InsertPoints(old_size, new_size, 0, point_sampler->GetOutput()->GetPoints());

  // create surface from new point set
  vtkSmartPointer<vtkSurfaceReconstructionFilter> surf = vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();

  surf->SetInputData(updated_mesh);

  vtkSmartPointer<vtkMarchingCubes> cf = vtkSmartPointer<vtkMarchingCubes>::New();

  cf->SetInputConnection(surf->GetOutputPort());
  cf->SetValue(0, 0.1);
  cf->ComputeNormalsOff();

  cf->Update();

  // Sometimes the contouring algorithm can create a volume whose gradient
  // vector and ordering of polygon (using the right hand rule) are
  // inconsistent. vtkReverseSense cures this problem.
  vtkSmartPointer<vtkReverseSense> reverse = vtkSmartPointer<vtkReverseSense>::New();
  reverse->SetInputConnection(cf->GetOutputPort());
  reverse->ReverseCellsOn();
  reverse->ReverseNormalsOn();
  reverse->Update();

  return reverse->GetOutput();
}

vtkSmartPointer<vtkPolyData> cutMesh(vtkSmartPointer<vtkPolyData>& mesh, vtkSmartPointer<vtkPoints>& points, bool get_inside)
{
  // create loop using the input points
  vtkSmartPointer<vtkImplicitSelectionLoop> loop = vtkSmartPointer<vtkImplicitSelectionLoop>::New();
  loop->AutomaticNormalGenerationOn();
  loop->SetLoop(points);

  // cut out a section of the mesh using the newly created loop
  // Note: the loop extends infitely along its central axis so multiple regions could be cut out
  vtkSmartPointer<vtkClipPolyData> clipper = vtkSmartPointer<vtkClipPolyData>::New();
  clipper->SetClipFunction(loop);
  clipper->SetInputData(mesh);
  if(get_inside)  // true, return inside of cut; false, return outside of cut
  {
    clipper->InsideOutOn(); // Off by default
  }
  clipper->Update();

  return clipper->GetOutput();
}


double pt_dist(double* pt1, double* pt2)
{
  return (pow(pt1[0] - pt2[0], 2) + pow(pt1[1] - pt2[1], 2 ) + pow((pt1[2] - pt2[2]), 2 ));
}

}
