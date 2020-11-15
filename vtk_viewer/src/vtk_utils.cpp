/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include "vtk_viewer/vtk_utils.h"
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>  // MESH conversion utils VTK<->PCL
#include <pcl/surface/mls.h>

#include <pcl/surface/grid_projection.h>  // surface reconstruction by Rosie Li based on:
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

#include <console_bridge/console.h>

namespace vtk_viewer
{
vtkSmartPointer<vtkPoints> createPlane(unsigned int grid_size_x, unsigned int grid_size_y, vtk_viewer::plane_type type)
{
  // Create points on an XY grid with a sinusoidal Z component
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  for (unsigned int x = 0; x < grid_size_x; x++)
  {
    for (unsigned int y = 0; y < grid_size_y; y++)
    {
      switch (type)
      {
        case vtk_viewer::FLAT:
          points->InsertNextPoint(x, y, vtkMath::Random(0.0, 0.001));
          break;
        case vtk_viewer::SINUSOIDAL_1D:
          points->InsertNextPoint(x, y, sin(double(y) / 2.0) + vtkMath::Random(0.0, 0.001));
          break;
        case vtk_viewer::SINUSOIDAL_2D:
          points->InsertNextPoint(
              x, y, 1.0 * cos(double(x) / 2.0) - 1.0 * sin(double(y) / 2.0) + vtkMath::Random(0.0, 0.001));
          break;
      }
    }
  }

  // Add the grid points to a polydata object
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  return points;
}

vtkSmartPointer<vtkPolyData> createMesh(vtkSmartPointer<vtkPoints> points, double sample_spacing, int neigborhood_size)
{
  if (!points)
  {
    points = createPlane();
  }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  // surface reconstruction
  vtkSmartPointer<vtkContourFilter> cf;
  vtkSmartPointer<vtkSurfaceReconstructionFilter> surf = vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();

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
  assert(cells->GetNumberOfCells() < std::numeric_limits<int>::max());
  int size = static_cast<int>(cells->GetNumberOfCells());
  // loop through all cells, mark cells for deletion if they do not have enough points nearby
  for (int i = 0; i < size; ++i)
  {
    // get cell centroid and size
    vtkCell* cell = mesh->GetCell(i);
    if (cell)
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
      double radius = 2.0 * sqrt(max);
      vtkSmartPointer<vtkIdList> pts = vtkSmartPointer<vtkIdList>::New();
      kd_tree->FindPointsWithinRadius(radius, center, pts);

      // if number of desired points not found, mark cell for deletion
      if (pts->GetNumberOfIds() < 6)
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

void visualizePlane(vtkSmartPointer<vtkPolyData>& polydata)
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
  renderer->SetBackground(.3, .6, .3);  // Background color green

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

bool loadPCDFile(std::string file, vtkSmartPointer<vtkPolyData>& polydata, std::string background, bool /*return_mesh*/)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file.c_str(), *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
    return false;
  }

  if (background != "")
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr bg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(background.c_str(), *bg_cloud) == -1)  //* load the file
    {
      PCL_ERROR("Couldn't read file \n");
    }
    else
    {
      // perform background subtraction
      removeBackground(*cloud, *bg_cloud);
    }
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr normals = vtk_viewer::pclEstimateNormals(cloud);
  pcl::PolygonMesh mesh = vtk_viewer::pclGridProjectionMesh(normals);
  vtk_viewer::pclEncodeMeshAndNormals(mesh, polydata);

  // vtkSurfaceReconstructionMesh(cloud, polydata);

  return true;
}

void vtkSurfaceReconstructionMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vtkSmartPointer<vtkPolyData>& mesh)
{
  // use MLS filter to smooth data and calculate normals (TODO: Normal data may not be oriented correctly)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
  pcl::PointCloud<pcl::PointXYZ> mls_points;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // Set parameters (TODO: Make these parameters configurable)
  mls.setComputeNormals(false);
  mls.setInputCloud(cloud);
  mls.setPolynomialOrder(2);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.01);

  // Reconstruct
  mls.process(mls_points);

  if (!mesh)
  {
    mesh = vtkSmartPointer<vtkPolyData>::New();
  }

  vtkSmartPointer<vtkPolyData> point_data = vtkSmartPointer<vtkPolyData>::New();
  PCLtoVTK(mls_points, point_data);

  // Mesh
  mesh = createMesh(point_data->GetPoints(), 0.01, 20);
  cleanMesh(point_data->GetPoints(), mesh);
}

void removeBackground(pcl::PointCloud<pcl::PointXYZ>& cloud, const pcl::PointCloud<pcl::PointXYZ>& background)
{
  if (cloud.points.size() != background.points.size())
  {
    return;
  }

  for (std::size_t i = 0; i < cloud.points.size(); ++i)
  {
    if (fabs(cloud.points[i].z - background.points[i].z) < 0.05f || std::isnan(background.points[i].z) ||
        cloud.points[i].y > 0.75f)
    {
      cloud.points[i].x = NAN;
      cloud.points[i].y = NAN;
      cloud.points[i].z = NAN;
    }
  }

  // create new point cloud with NaN's removed
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  for (std::size_t i = 0; i < cloud.points.size(); ++i)
  {
    if (!std::isnan(cloud.points[i].x))
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

void PCLtoVTK(const pcl::PointCloud<pcl::PointXYZ>& cloud, vtkPolyData* const pdata)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  for (std::size_t i = 0; i < cloud.points.size(); ++i)
  {
    if (std::isnan(cloud.points[i].x) || std::isnan(cloud.points[i].y) || std::isnan(cloud.points[i].z))
    {
      continue;
    }
    const double pt[3] = { double(cloud.points[i].x), double(cloud.points[i].y), double(cloud.points[i].z) };
    points->InsertNextPoint(pt);
  }

  // Create a temporary PolyData and add the points to it
  vtkSmartPointer<vtkPolyData> tempPolyData = vtkSmartPointer<vtkPolyData>::New();
  tempPolyData->SetPoints(points);

  pdata->DeepCopy(tempPolyData);
}

void VTKtoPCL(vtkPolyData* const pdata, pcl::PointCloud<pcl::PointNormal>& cloud)
{
  for (long i = 0; i < pdata->GetPoints()->GetNumberOfPoints(); ++i)
  {
    pcl::PointNormal pt;
    double* ptr = pdata->GetPoints()->GetPoint(i);
    pt.x = static_cast<float>(ptr[0]);
    pt.y = static_cast<float>(ptr[1]);
    pt.z = static_cast<float>(ptr[2]);

    double* norm = pdata->GetPointData()->GetNormals()->GetTuple(i);
    pt.normal_x = static_cast<float>(norm[0]);
    pt.normal_y = static_cast<float>(norm[1]);
    pt.normal_z = static_cast<float>(norm[2]);

    cloud.push_back(pt);
  }
}

vtkSmartPointer<vtkPolyData> estimateCurvature(vtkSmartPointer<vtkPolyData> mesh, int method)
{
  vtkSmartPointer<vtkCurvatures> curvature_filter = vtkSmartPointer<vtkCurvatures>::New();
  curvature_filter->SetInputData(mesh);
  switch (method)
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

bool embedRightHandRuleNormals(vtkSmartPointer<vtkPolyData>& data)
{
  bool success = true;
  CONSOLE_BRIDGE_logDebug("Embedding mesh normals from right hand rule");
  int size = static_cast<int>(data->GetNumberOfCells());
  vtkDoubleArray* cell_normals = vtkDoubleArray::New();

  cell_normals->SetNumberOfComponents(3);
  cell_normals->SetNumberOfTuples(size);

  // Counter for cells that are malformed
  unsigned long bad_cells = 0;

  // loop through all cells and add cell normals
  for (int i = 0; i < size; ++i)
  {
    vtkCell* cell = data->GetCell(i);
    if (cell)
    {
      // Get all point IDs associated with the given cell
      vtkIdList* pts = cell->GetPointIds();
      double norm[3] = { 0, 0, 0 };

      // If there are at least 3 points associated with the cell
      if (pts->GetNumberOfIds() >= 3)
      {
        // Define arrays to store the points
        double p0[3] = { 1, 0, 0 };
        double p1[3] = { 0, 1, 0 };
        double p2[3] = { 0, 0, 1 };

        // Extract points from polydata that are associated with this cell
        data->GetPoint(pts->GetId(0), p0);
        data->GetPoint(pts->GetId(1), p1);
        data->GetPoint(pts->GetId(2), p2);

        // Get vectors obeying RHR
        double v1[3];
        double v2[3];
        double v3[3];
        for (int ind = 0; ind < 3; ind++)
        {
          v1[ind] = p1[ind] - p0[ind];
          v2[ind] = p2[ind] - p1[ind];
          v3[ind] = p0[ind] - p2[ind];
        }
        // Normalize the vectors
        vtkMath::Normalize(v1);
        vtkMath::Normalize(v2);
        vtkMath::Normalize(v3);

        // Make sure the angle between the vectors fairly large (dot product > threshold)
        if (std::abs(v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) > 0.707)
        {
          // Normal is v1 x v2
          norm[0] = v1[1] * v2[2] - v2[1] * v1[2];
          norm[1] = -1 * (v1[0] * v2[2] - v2[0] * v1[2]);
          norm[2] = v1[0] * v2[1] - v2[0] * v1[1];
        }
        else
        {
          // Normal is v2 x v3
          norm[0] = v2[1] * v3[2] - v3[1] * v2[2];
          norm[1] = -1 * (v2[0] * v3[2] - v3[0] * v2[2]);
          norm[2] = v2[0] * v3[1] - v3[0] * v2[1];
        }

        // Normalize the normals
        vtkMath::Normalize(norm);

        // set the normal for the given cell
        cell_normals->SetTuple(i, norm);
      }
      else
      {
        bad_cells++;
        success = false;
      }
    }
  }
  if (bad_cells > 0)
    CONSOLE_BRIDGE_logError("Could not embed normals on %d cells", bad_cells);

  // We have looped over every cell. Now embed the normals
  data->GetCellData()->SetNormals(cell_normals);
  return success;
}

void generateNormals(vtkSmartPointer<vtkPolyData>& data, int flip_normals)
{
  // If point data exists but cell data does not, iterate through the cells and generate normals manually
  if (data->GetPointData()->GetNormals() && !data->GetCellData()->GetNormals())
  {
    CONSOLE_BRIDGE_logDebug("Generating Mesh Normals manually");
    int size = static_cast<int>(data->GetNumberOfCells());
    vtkDoubleArray* cell_normals = vtkDoubleArray::New();

    cell_normals->SetNumberOfComponents(3);
    cell_normals->SetNumberOfTuples(size);

    // loop through all cells and add cell normals
    for (int i = 0; i < size; ++i)
    {
      vtkCell* cell = data->GetCell(i);
      if (cell)
      {
        // Get all point normals associated with the given cell
        vtkIdList* pts = cell->GetPointIds();
        double norm[3] = { 0, 0, 0 };
        for (int j = 0; j < pts->GetNumberOfIds(); ++j)
        {
          double p0[3];
          data->GetPointData()->GetNormals()->GetTuple(pts->GetId(j), p0);

          // Based on vtk's api it should return 0 if there is an error
          if (vtkMath::Normalize(p0) > 0.0)
          {
            norm[0] += p0[0];
            norm[1] += p0[1];
            norm[2] += p0[2];
          }
        }

        if (vtkMath::Normalize(norm) <= 0.0)
        {
          PCL_ERROR("Could not calculate cell normal from point normals!\n");
        }

        // set the normal for the given cell
        cell_normals->SetTuple(i, norm);
      }
    }
    data->GetCellData()->SetNormals(cell_normals);
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Recomputing Mesh normals");
    vtkSmartPointer<vtkPolyDataNormals> normal_generator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normal_generator->SetInputData(data);
    normal_generator->ComputePointNormalsOn();
    normal_generator->ComputeCellNormalsOn();

    // Optional settings
    normal_generator->SetFeatureAngle(M_PI_2);
    normal_generator->SetSplitting(0);
    normal_generator->SetConsistency(1);
    normal_generator->SetAutoOrientNormals(flip_normals);

    if (!data->GetPointData()->GetNormals())
    {
      normal_generator->SetComputePointNormals(1);
      CONSOLE_BRIDGE_logDebug("Point Normals Computation ON");
    }
    else
    {
      normal_generator->SetComputePointNormals(0);
      CONSOLE_BRIDGE_logDebug("Point Normals Computation OFF");
    }

    if (!data->GetCellData()->GetNormals())
    {
      normal_generator->SetComputeCellNormals(1);
      CONSOLE_BRIDGE_logDebug("Cell Normals Computation ON");
    }
    else
    {
      normal_generator->SetComputeCellNormals(0);
      CONSOLE_BRIDGE_logDebug("Cell Normals Computation OFF");
    }

    normal_generator->SetFlipNormals(0);
    normal_generator->SetNonManifoldTraversal(0);

    normal_generator->Update();
    vtkDataArray* normals = normal_generator->GetOutput()->GetPointData()->GetNormals();
    if (normals)
    {
      data->GetPointData()->SetNormals(normals);
    }

    vtkDataArray* normals2 = normal_generator->GetOutput()->GetCellData()->GetNormals();
    if (normals2)
    {
      data->GetCellData()->SetNormals(normals2);
    }
  }
}

vtkSmartPointer<vtkPolyData> sampleMesh(vtkSmartPointer<vtkPolyData> mesh, double distance)
{
  // Sample the mesh.  Points can be closer than `distance` since it also includes vertices
  vtkSmartPointer<vtkPolyDataPointSampler> point_sampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
  point_sampler->SetDistance(distance);
  point_sampler->SetInputData(mesh);
  point_sampler->Update();

  // Clean up polydata to remove points that are too close.  Since Tolerance is range [0, 1.0],
  // we need to get the mesh extents to determine what the tolerance value is to get the right point spacing
  double bounds[6];
  mesh->GetBounds(bounds);
  double x = bounds[1] - bounds[0];
  double y = bounds[3] - bounds[2];
  double z = bounds[5] - bounds[4];
  double max = x > y ? x : y;
  max = max > z ? max : z;

  double tolerance = distance / max;

  vtkSmartPointer<vtkCleanPolyData> clean_polydata = vtkSmartPointer<vtkCleanPolyData>::New();
  clean_polydata->SetInputData(point_sampler->GetOutput());
  clean_polydata->SetTolerance(tolerance);
  clean_polydata->Update();

  return clean_polydata->GetOutput();
}

vtkSmartPointer<vtkPolyData> cutMesh(vtkSmartPointer<vtkPolyData>& mesh,
                                     vtkSmartPointer<vtkPoints>& points,
                                     bool get_inside)
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
  if (get_inside)  // true, return inside of cut; false, return outside of cut
  {
    clipper->InsideOutOn();  // Off by default
  }
  clipper->Update();

  return clipper->GetOutput();
}

double pt_dist(double* pt1, double* pt2)
{
  return (pow(pt1[0] - pt2[0], 2) + pow(pt1[1] - pt2[1], 2) + pow((pt1[2] - pt2[2]), 2));
}

/**
 * @brief Internal helper to produce normals
 */
static pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                         double radius,
                                                         const pcl::PointXYZ& view_point)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> netmp;
  netmp.setInputCloud(cloud);
  netmp.setViewPoint(view_point.x, view_point.y, view_point.z);
  netmp.setRadiusSearch(radius);
  netmp.compute(*normals);

  return normals;
}

pcl::PointCloud<pcl::PointNormal>::Ptr pclEstimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                          double radius,
                                                          const pcl::PointXYZ& view_point)
{
  // Compute normals and add to original points
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2tmp = estimateNormals(cloud, radius, view_point);

  pcl::PointCloud<pcl::PointNormal>::Ptr new_cloudtmp(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud, *cloud_normals2tmp, *new_cloudtmp);

  return new_cloudtmp;
}

pcl::PolygonMesh pclGridProjectionMesh(pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,
                                       double resolution,
                                       int padding_size,
                                       int max_binary_searc_level,
                                       int nearest_neighbors)
{
  // Perform Grid Projection point cloud meshing (Polygonizing Extremal Surfaces with Manifold Guarantees, Ruosi Li,
  // et. al.)
  pcl::GridProjection<pcl::PointNormal> grid_surf;
  pcl::PolygonMesh output_mesh;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud);

  // Set parameters
  grid_surf.setResolution(resolution);  // this parameter is the main one which affects the smoothness of the mesh
  grid_surf.setPaddingSize(padding_size);
  grid_surf.setMaxBinarySearchLevel(max_binary_searc_level);  // default is 10
  grid_surf.setNearestNeighborNum(nearest_neighbors);         // default is 50
  grid_surf.setSearchMethod(tree2);

  // Mesh
  grid_surf.setInputCloud(cloud);
  grid_surf.reconstruct(output_mesh);
  return output_mesh;
}

void pclEncodeMeshAndNormals(const pcl::PolygonMesh& pcl_mesh,
                             vtkSmartPointer<vtkPolyData>& vtk_mesh,
                             double radius,
                             const pcl::PointXYZ& view_point)
{
  pcl::PolygonMesh copy = pcl_mesh;

  // Convert pcl::PCLPointCloud2 -> pcl::PointCloud<T>
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(copy.cloud, *temp_cloud);

  // Compute normals and add to original points
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 = estimateNormals(temp_cloud, radius, view_point);

  pcl::PointCloud<pcl::PointNormal>::Ptr new_cloud(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*temp_cloud, *cloud_normals2, *new_cloud);

  // Convert pcl::PointCloud<PointNormal> -> pcl::PCLPointCloud2 and store in original mesh object
  pcl::toPCLPointCloud2(*new_cloud, copy.cloud);

  // Convert mesh object to VTK
  pcl::VTKUtils::convertToVTK(copy, vtk_mesh);
}

bool loadPolygonMeshFromPLY(const std::string& file, pcl::PolygonMesh& mesh)
{
  if (pcl::io::loadPolygonFilePLY(file, mesh) == -1)
  {
    PCL_ERROR("Couldn't read file %s\n", file.c_str());
    return false;
  }
  return true;
}
}  // namespace vtk_viewer
