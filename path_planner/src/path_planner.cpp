/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <Eigen/Core>
#include <Eigen/Dense>

#include <path_planner/path_planner.h>
#include <limits>

#include <vtkIntersectionPolyDataFilter.h>
#include <vtkDelaunay2D.h>
#include <vtkMath.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkCellLinks.h>
#include <vtkPolyDataNormals.h>
#include <vtkKdTreePointLocator.h>

#include <vtkDoubleArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>

#include <vtk_viewer/vtk_utils.h>

using namespace vtk_viewer;

namespace path_planner
{
  vtkSmartPointer<vtkPolyData> PathPlanner::getFirstPath()
  {
    // create vertical plane
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    unsigned int gridSize = 10;
    for(unsigned int x = 0; x < gridSize; x++)
      {
      for( int z = -5; z < 5; z++)
        {
        points->InsertNextPoint( x , (5 - z + vtkMath::Random(0.0, 0.1)) , z);
        }
      }

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> polydata2 = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);
    vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
    delaunay->SetInputData(polydata);
    delaunay->Update();

    delaunay->SetAlpha(3);
    delaunay->SetOffset(3);
    //delaunay->SetProjectionPlaneMode(3);
    polydata2 = delaunay->GetOutput();

    // use Intersection filter
    vtkSmartPointer<vtkIntersectionPolyDataFilter> intersectionPolyDataFilter =
      vtkSmartPointer<vtkIntersectionPolyDataFilter>::New();
    intersectionPolyDataFilter->SetInputData( 0, _input_mesh);
    intersectionPolyDataFilter->SetInputData( 1, polydata2 );
    intersectionPolyDataFilter->Update();

    // return results
    return intersectionPolyDataFilter->GetOutput();

  }

  vtkSmartPointer<vtkPolyData> PathPlanner::smoothData(vtkSmartPointer<vtkPoints> points, vtkSmartPointer<vtkPolyData>& derivatives)
  {
    // Sort points and create spline
    vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
    points2 = sortPoints(points);
    vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
    spline->SetPoints(points2);

    vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> points3 = vtkSmartPointer<vtkPoints>::New();

    vtkSmartPointer<vtkDoubleArray> derv = vtkSmartPointer<vtkDoubleArray>::New();
    derv->SetNumberOfComponents(3);

    double new_pt[3] = {0, 0, 0};
    double* new_ptr;
    new_ptr = &new_pt[0];

    //int max = output->GetPoints()->GetNumberOfPoints();
    // get points which are evenly spaced along the spline
    int max = 20;
    double u[3], pt[3], d[9];
    for( int i = 0; i <= max; ++i)
    {

      u[0] = double(i)/double(max);
      u[1] = double(i)/double(max);
      u[2] = double(i)/double(max);
      spline->Evaluate(u, pt, d);
      points3->InsertNextPoint(pt);

      double pt1[3], pt2[3];
      // find nearby point in order to calculate the local deriviative
      if(i < max)
      {
        pt1[0] = pt[0];
        pt1[1] = pt[1];
        pt1[2] = pt[2];
        u[0] += 1/(2*double(max));
        spline->Evaluate(u, pt2, d);
      }
      else
      {
        pt2[0] = pt[0];
        pt2[1] = pt[1];
        pt2[2] = pt[2];
        u[0] -= 1/(2*double(max));
        spline->Evaluate(u, pt1, d);
      }

      // calculate the derivative
      new_ptr[0] = pt1[0] - pt2[0];
      new_ptr[1] = pt1[1] - pt2[1];
      new_ptr[2] = pt1[2] - pt2[2];
      double der = sqrt(pow(new_ptr[0],2) + pow(new_ptr[1],2) + pow(new_ptr[2],2));

      // normalize the derivative vector
      new_ptr[0] /= der;
      new_ptr[1] /= der;
      new_ptr[2] /= der;

      derv->InsertNextTuple(new_ptr);
    }
    output->SetPoints(points3);
    derivatives->SetPoints(points3);
    derivatives->GetPointData()->SetNormals(derv);

    return output;
  }

  // Sort points in linear order
  vtkSmartPointer<vtkPoints> PathPlanner::sortPoints(vtkSmartPointer<vtkPoints>& points)
  {
    std::vector<std::vector<double> > new_points;
    double d[3];
    for(int i = 0; i < points->GetNumberOfPoints(); ++i)
    {
        points->GetPoint(i, d);
        std::vector<double> pt;
        pt.push_back(d[0]);
        pt.push_back(d[1]);
        pt.push_back(d[2]);
        new_points.push_back(pt);
    }

    std::vector<std::vector<double> > sorted_points;

    // create new vector of sorted points
    int size = new_points.size();
    while(sorted_points.size() != size)
    {
      if(sorted_points.size() == 0)
      {
        sorted_points.push_back(new_points.front());
        new_points.erase(new_points.begin());
      }
      else
      {
        int next = findClosestPoint(sorted_points.back(), new_points);
        if (dist(sorted_points.back(), new_points[next]) < dist(sorted_points.front(), new_points[next]))
        {
          sorted_points.push_back(new_points[next]);
          new_points.erase(new_points.begin() + next);
        }
        else
        {
          next = findClosestPoint(sorted_points.front(), new_points);
          sorted_points.insert(sorted_points.begin(), new_points[next]);
          new_points.erase(new_points.begin() + next);
        }
      }
    }

    // convert array to vtkPoints and return
    vtkSmartPointer<vtkPoints> line_points = vtkSmartPointer<vtkPoints>::New();
    for(int i = 0; i < size; ++i)
    {
      line_points->InsertNextPoint(sorted_points[i][0], sorted_points[i][1], sorted_points[i][2]);
    }

    return line_points;
  }

  int PathPlanner::findClosestPoint(std::vector<double>& pt,  std::vector<std::vector<double> >& pts)
  {
    double min = std::numeric_limits<double>::max();
    int index = -1;
    for(int i = 0; i < pts.size(); ++i)
    {
      double d = dist(pt, pts[i]);
      if(d < min)
      {
        index = i;
        min = d;
      }
    }
    return index;
  }

  double PathPlanner::dist(std::vector<double>& pt1, std::vector<double>& pt2)
  {
    return (pow(pt1[0] - pt2[0], 2) + pow(pt1[1] - pt2[1], 2 ) + pow((pt1[2] - pt2[2]), 2 ));
  }

  vtkSmartPointer<vtkPolyData> PathPlanner::generateNormals(vtkSmartPointer<vtkPolyData> data)
  {
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator->SetInputData(data);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOn();

    // Optional settings
    normalGenerator->SetFeatureAngle(0.1);
    normalGenerator->SetSplitting(0);
    normalGenerator->SetConsistency(1);
    normalGenerator->SetAutoOrientNormals(0);
    normalGenerator->SetComputePointNormals(1);
    normalGenerator->SetComputeCellNormals(0);
    normalGenerator->SetFlipNormals(1);
    normalGenerator->SetNonManifoldTraversal(1);

    normalGenerator->Update();

    polydata = normalGenerator->GetOutput();

    vtkDataArray* normalsGeneric = polydata->GetPointData()->GetNormals();
      if(normalsGeneric)
      {
        cout << "normals exist\n" ;
        std::cout << "There are " << normalsGeneric->GetNumberOfTuples()
                          << " point normals." << std::endl;
//        for(int i = 0; i < normalsGeneric->GetNumberOfTuples(); ++i)
//        {
//          double* val = normalsGeneric->GetTuple(i);
//          //double vals = *val;
//          cout << val[0] << " " << val[1] << " " << val[2] << "\n";
//        }
      }

      _input_mesh->GetPointData()->SetNormals(normalsGeneric);

//    vtkSmartPointer<vtkDoubleArray> pointNormalsRetrieved =
//        vtkDoubleArray::SafeDownCast(polydata->GetPolys()->GetData());
//      if(pointNormalsRetrieved)
//        {
//        std::cout << "There are " << pointNormalsRetrieved->GetNumberOfTuples()
//                  << " point normals." << std::endl;

//      }

    return polydata;

  }

  void PathPlanner::estimateNewNormals(vtkSmartPointer<vtkPolyData>& data)
  {
    //Create the tree
    vtkSmartPointer<vtkKdTreePointLocator> pointTree =
        vtkSmartPointer<vtkKdTreePointLocator>::New();
    pointTree->SetDataSet(_input_mesh);
    pointTree->BuildLocator();

    // Find k nearest neighbors and use their normals to estimate the normal of the desired point
    unsigned int k = 5;

    vtkSmartPointer<vtkDoubleArray> new_norm = vtkSmartPointer<vtkDoubleArray>::New();
    new_norm->SetNumberOfComponents(3);

    for(int i = 0; i < data->GetPoints()->GetNumberOfPoints(); ++i)
    {
      double* pt = data->GetPoints()->GetPoint(i);
      double testPoint[3] = {pt[0], pt[1], pt[2]};
      vtkSmartPointer<vtkIdList> result = vtkSmartPointer<vtkIdList>::New();

      pointTree->FindClosestNPoints(k, testPoint, result);
      double final[3] = {0, 0, 0};
      int count = 0;
      for(int j = 0; j < result->GetNumberOfIds(); ++j)
      {
        //cout << final[0] << " " << final[1] << " " << final[2] << "\n";
        pt = _input_mesh->GetPointData()->GetNormals()->GetTuple(result->GetId(j));
        if(isnan(pt[0]) || isnan(pt[1]) || isnan(pt[2]))
        {
          continue;
        }
        final[0] += pt[0];
        final[1] += pt[1];
        final[2] += pt[2];
        ++count;
      }

      double* pt2 = pt;

      if(count == 0)
      {
        pt2[0] = 0;
        pt2[1] = 0;
        pt2[2] = 0;
      }
      else
      {
        final[0] /= double(count);
        final[1] /= double(count);
        final[2] /= double(count);
        pt2[0] = final[0];
        pt2[1] = final[1];
        pt2[2] = final[2];
      }
      new_norm->InsertNextTuple(pt2);
    }
    data->GetPointData()->SetNormals(new_norm);

  }

  vtkSmartPointer<vtkPolyData> PathPlanner::createOffsetLine(vtkSmartPointer<vtkPolyData> line, vtkSmartPointer<vtkPolyData> derivatives, double dist)
  {
    vtkSmartPointer<vtkPolyData> new_points;

    vtkDataArray* normals = line->GetPointData()->GetNormals();
    vtkDataArray* ders = derivatives->GetPointData()->GetNormals();
    if(!normals || !ders)
    {
      // return null pointer
      return new_points;
    }

    if(normals->GetNumberOfTuples() != ders->GetNumberOfTuples())
    {
      return new_points;
    }

    new_points = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> new_pts = vtkSmartPointer<vtkPoints>::New();

    // calculate offset for each point
    for(int i = 0; i < normals->GetNumberOfTuples(); ++i)
    {
      //calculate cross to get offset direction
      double* pt1 = normals->GetTuple(i);
      double* pt2 = ders->GetTuple(i);

      Eigen::Vector3d u(pt1[0], pt1[1], pt1[2]);
      Eigen::Vector3d v(pt2[0], pt2[1], pt2[2]);
      Eigen::Vector3d w = u.cross(v);

      // use point, direction w, and dist, to create new point
      double new_pt[3];
      double* pt = line->GetPoints()->GetPoint(i);
      new_pt[0] = pt[0] + w[0] * dist;
      new_pt[1] = pt[1] + w[1] * dist;
      new_pt[2] = pt[2] + w[2] * dist;

      new_pts->InsertNextPoint(new_pt);
    }
    new_points->SetPoints(new_pts);

    return new_points;
  }

  vtkSmartPointer<vtkPolyData> PathPlanner::createSurfaceFromSpline(vtkSmartPointer<vtkPolyData> line, double dist)
  {
    vtkSmartPointer<vtkPolyData> new_surface;
    vtkSmartPointer<vtkDataArray> normals = line->GetPointData()->GetNormals();

    if(!normals)
    {
      return new_surface;
    }

    new_surface = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

    // for each point, insert 2 points, one above and one below, to create a new surface
    for(int i = 0; i < normals->GetNumberOfTuples(); ++i)
    {
      double* norm = normals->GetTuple(i);
      double* pt = line->GetPoints()->GetPoint(i);

      double new_pt[3];
      new_pt[0] = pt[0] + norm[0] * dist;
      new_pt[1] = pt[1] + norm[1] * dist;
      new_pt[2] = pt[2] + norm[2] * dist;

      points->InsertNextPoint(new_pt);

      new_pt[0] = pt[0] - norm[0] * dist;
      new_pt[1] = pt[1] - norm[1] * dist;
      new_pt[2] = pt[2] - norm[2] * dist;

      points->InsertNextPoint(new_pt);

      points->InsertNextPoint(pt);
    }

    new_surface = vtk_viewer::createMesh(points);
    new_surface->SetPoints(points);

  }
}
