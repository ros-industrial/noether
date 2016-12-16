/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include "vtk_viewer/vtk_utils.h"

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/surface/mls.h>

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
#include <vtkMarchingContourFilter.h>
#include <vtkMarchingCubes.h>
#include <vtkCurvatures.h>

#include <vtkSurfaceReconstructionFilter.h>
#include <vtkReverseSense.h>
#include <vtkContourFilter.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkKdTreePointLocator.h>
#include <vtkTriangle.h>
#include <vtkCleanPolyData.h>


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
  vtkSmartPointer<vtkReverseSense> reverse =
    vtkSmartPointer<vtkReverseSense>::New();
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
      double radius =  sqrt(max);
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
  vtkSmartPointer<vtkCleanPolyData> cleanPolyData =
      vtkSmartPointer<vtkCleanPolyData>::New();
  cleanPolyData->SetInputData(mesh);
  //cleanPolyData->SetTolerance(0.1);
  cleanPolyData->Update();

  mesh = cleanPolyData->GetOutput();

}

void visualizePlane(vtkSmartPointer<vtkPolyData> &polydata)
{
  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> triangulatedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  triangulatedMapper->SetInputData(polydata);

  vtkSmartPointer<vtkActor> triangulatedActor = vtkSmartPointer<vtkActor>::New();
  triangulatedActor->SetMapper(triangulatedMapper);

  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actor to the scene
  renderer->AddActor(triangulatedActor);
  renderer->SetBackground(.3, .6, .3); // Background color green

  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
}

vtkSmartPointer<vtkPolyData> readSTLFile(std::string file)
{
  vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
  reader->SetFileName(file.c_str());
  reader->SetMerging(1);
  reader->Update();

  return reader->GetOutput();
}

bool loadPCDFile(std::string file, vtkSmartPointer<vtkPolyData>& polydata)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file.c_str(), *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file \n");
      return false;
    }

  // smooth the data
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointXYZ> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

  mls.setComputeNormals (false);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (mls_points);

  polydata = vtkSmartPointer<vtkPolyData>::New();
  PCLtoVTK(mls_points, polydata);

  return true;
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
      removeBackground(*cloud, *bg_cloud);
    }
  }

  // smooth the data
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointXYZ> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

  mls.setComputeNormals (false);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (mls_points);

  polydata = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPolyData> point_data = vtkSmartPointer<vtkPolyData>::New();
  PCLtoVTK(mls_points, point_data);

  if(return_mesh)
  {
    polydata = createMesh(point_data->GetPoints(), 0.02, 30);
    cleanMesh(point_data->GetPoints(), polydata);
  }
  else
  {
    polydata = point_data;
  }

  return true;
}

void removeBackground(pcl::PointCloud<pcl::PointXYZ>& cloud, const pcl::PointCloud<pcl::PointXYZ>& background)
{
  if(cloud.points.size() != background.points.size())
  {
    return;
  }

  for (int i = 0; i < cloud.points.size(); ++i)
  {
    if(fabs(cloud.points[i].z - background.points[i].z) < 0.05 || isnan(background.points[i].z) || cloud.points[i].y > 0.75)
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
    if(!isnan(cloud.points[i].x))
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
    if(isnan(cloud.points[i].x) || isnan(cloud.points[i].y) || isnan(cloud.points[i].z))
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

vtkSmartPointer<vtkPolyData> estimateCurvature(vtkSmartPointer<vtkPolyData> mesh, int method)
{
  vtkSmartPointer<vtkCurvatures> curvaturesFilter =
      vtkSmartPointer<vtkCurvatures>::New();
    curvaturesFilter->SetInputData(mesh);
    curvaturesFilter->SetCurvatureTypeToMean();
    switch(method)
    {
    case 0:
      curvaturesFilter->SetCurvatureTypeToMinimum();
      break;
    case 1:
      curvaturesFilter->SetCurvatureTypeToMaximum();
      break;
    case 2:
      curvaturesFilter->SetCurvatureTypeToGaussian();
      break;
    case 3:
    default:
      curvaturesFilter->SetCurvatureTypeToMean();
      break;
    }
    curvaturesFilter->Update();

    return curvaturesFilter->GetOutput();
}

void generateNormals(vtkSmartPointer<vtkPolyData>& data)
{
  vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
  normalGenerator->SetInputData(data);
  normalGenerator->ComputePointNormalsOn();
  normalGenerator->ComputeCellNormalsOn();

  // Optional settings
  normalGenerator->SetFeatureAngle(0.1);
  normalGenerator->SetSplitting(0);
  normalGenerator->SetConsistency(1);
  normalGenerator->SetAutoOrientNormals(1);
  normalGenerator->SetComputePointNormals(1);
  normalGenerator->SetComputeCellNormals(1);
  normalGenerator->SetFlipNormals(0);
  normalGenerator->SetNonManifoldTraversal(1);

  normalGenerator->Update();

  vtkDataArray* normals = normalGenerator->GetOutput()->GetPointData()->GetNormals();
  if(normals)
  {
    data->GetPointData()->SetNormals(normals);
  }

  vtkDataArray* normals2 = normalGenerator->GetOutput()->GetCellData()->GetNormals();
  if(normals2)
  {
    data->GetCellData()->SetNormals(normals2);
  }
}

vtkSmartPointer<vtkPolyData> upsampleMesh(vtkSmartPointer<vtkPolyData> mesh, double distance)
{
  vtkSmartPointer<vtkPolyDataPointSampler> pointSampler =
      vtkSmartPointer<vtkPolyDataPointSampler>::New();
  pointSampler->SetDistance(distance);
  pointSampler->SetInputData(mesh);
  pointSampler->Update();

  // Resize the mesh and insert the new points
  int old_size = mesh->GetPoints()->GetNumberOfPoints();
  int new_size = pointSampler->GetOutput()->GetPoints()->GetNumberOfPoints();

  vtkSmartPointer<vtkPolyData> updated_mesh = vtkSmartPointer<vtkPolyData>::New();
  updated_mesh->SetPoints(mesh->GetPoints());
  int size = old_size + new_size;
  updated_mesh->GetPoints()->Resize(size);

  updated_mesh->GetPoints()->InsertPoints(old_size, new_size, 0, pointSampler->GetOutput()->GetPoints());

  // create surface from new point set
  vtkSmartPointer<vtkSurfaceReconstructionFilter> surf =
      vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();

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

double pt_dist(double* pt1, double* pt2)
{
  return (pow(pt1[0] - pt2[0], 2) + pow(pt1[1] - pt2[1], 2 ) + pow((pt1[2] - pt2[2]), 2 ));
}

}
