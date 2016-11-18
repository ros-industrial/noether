/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <Eigen/Core>
#include <Eigen/Dense>

#include <tool_path_planner/tool_path_planner.h>
#include <limits>

#include <vtkIntersectionPolyDataFilter.h>
#include <vtkDelaunay2D.h>
#include <vtkMath.h>
#include <vtkSpline.h>
#include <vtkPolyDataNormals.h>
#include <vtkKdTreePointLocator.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtk_viewer/vtk_utils.h>



namespace tool_path_planner
{

  double pt_dist(double* pt1, double* pt2)
  {
    return (pow(pt1[0] - pt2[0], 2) + pow(pt1[1] - pt2[1], 2 ) + pow((pt1[2] - pt2[2]), 2 ));
  }

  double calc_distance(std::vector<double>& pt1, std::vector<double>& pt2)
  {
    return (pow(pt1[0] - pt2[0], 2) + pow(pt1[1] - pt2[1], 2 ) + pow((pt1[2] - pt2[2]), 2 ));
  }

  int findClosestPoint(std::vector<double>& pt,  std::vector<std::vector<double> >& pts)
  {
    double min = std::numeric_limits<double>::max();
    int index = -1;
    for(int i = 0; i < pts.size(); ++i)
    {
      double d = calc_distance(pt, pts[i]);
      if(d < min)
      {
        index = i;
        min = d;
      }
    }
    return index;
  }

  void ToolPathPlanner::setInputMesh(vtkSmartPointer<vtkPolyData> mesh)
  {
    input_mesh_ = mesh;
    generateNormals(input_mesh_);
  }

  bool ToolPathPlanner::computePaths()
  {
    // Need to call getFirstPath or other method to generate the first path
    // If no paths exist, there is nothing to create offset paths from
    if(paths_.size() != 1)
    {
      return false;
    }

    // could potentially make these two loops run in parallel but then we would need to add locks on the data
    bool done = false;
    int max = 10;
    int count = 0;

    // From existing cutting plane, create more offset planes in one direction
    while(!done && count < max)
    {
      ProcessPath path2;
      if(getNextPath(paths_.back(), path2, tool_.line_spacing))
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
      if(getNextPath(paths_.front(), path2, -tool_.line_spacing))
      {
        paths_.insert(paths_.begin(), path2 );
      }
      else
      {
        done = true;
      }
      ++count;
    }
    return true;
  }

  void ToolPathPlanner::getFirstPath(ProcessPath& path)
  {
    // create vertical plane
    vtkSmartPointer<vtkPoints> mesh_points = vtkSmartPointer<vtkPoints>::New();
    unsigned int gridSize = 10;
    for(unsigned int x = 0; x < gridSize; x++)
      {
      for( int z = -5; z < 5; z++)
        {
        mesh_points->InsertNextPoint( x , (5 - z + vtkMath::Random(0.0, 0.1)) , z);
        }
      }

    // using points create cutting mesh
    vtkSmartPointer<vtkPolyData> cutting_mesh = vtk_viewer::createMesh(mesh_points);

    // use cutting mesh to find intersection line
    vtkSmartPointer<vtkPolyData> intersection_line = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();

    findIntersectionLine(cutting_mesh, intersection_line, spline);

    //use spline to create interpolated data with normals and derivatives
    vtkSmartPointer<vtkPolyData> points = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> derivatives = vtkSmartPointer<vtkPolyData>::New();
    smoothData(spline, points, derivatives);

    path.line = points;
    path.spline = spline;
    path.derivatives = derivatives;
    path.intersection_plane = cutting_mesh;

    paths_.push_back(path);
  }



  bool ToolPathPlanner::getNextPath(const ProcessPath this_path, ProcessPath& next_path, double dist)
  {

    // create offset points
    vtkSmartPointer<vtkPolyData> offset_line = vtkSmartPointer<vtkPolyData>::New();
    offset_line = createOffsetLine(this_path.line, this_path.derivatives, dist);

    // create cutting surface  TODO: offset may need to be based upon point bounds
    next_path.intersection_plane = createSurfaceFromSpline(offset_line, tool_.intersecting_plane_height);

    // use surface to find intersection line
    vtkSmartPointer<vtkPolyData> intersection_line = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();

    if(!findIntersectionLine(next_path.intersection_plane, intersection_line, spline))
    {
      return false;
    }

    //use spline to create interpolated data with normals and derivatives
    vtkSmartPointer<vtkPolyData> points = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> derivatives = vtkSmartPointer<vtkPolyData>::New();
    smoothData(spline, points, derivatives);

    next_path.line = points;
    next_path.spline = spline;
    next_path.derivatives = derivatives;

    // compare start/end points of new line to old line, flip order if necessary
    int length = next_path.line->GetPoints()->GetNumberOfPoints();
    if(pt_dist(this_path.line->GetPoints()->GetPoint(0), next_path.line->GetPoints()->GetPoint(0))
       > pt_dist(this_path.line->GetPoints()->GetPoint(0), next_path.line->GetPoints()->GetPoint(length-1)))
    {
      flipPointOrder(next_path);
    }

    return true;
  }

  bool ToolPathPlanner::findIntersectionLine(vtkSmartPointer<vtkPolyData> cut_surface,
                                         vtkSmartPointer<vtkPolyData>& points,
                                         vtkSmartPointer<vtkParametricSpline>& spline)
  {
    // Get intersection line
    vtkSmartPointer<vtkIntersectionPolyDataFilter> intersectionPolyDataFilter =
      vtkSmartPointer<vtkIntersectionPolyDataFilter>::New();
    intersectionPolyDataFilter->SetInputData( 0, input_mesh_);
    intersectionPolyDataFilter->SetInputData( 1, cut_surface );
    intersectionPolyDataFilter->Update();

    // Sort points
    vtkSmartPointer<vtkPoints> pts = intersectionPolyDataFilter->GetOutput()->GetPoints();
    if(!pts || pts->GetNumberOfPoints() == 0)
    {
      return false;
    }
    sortPoints(pts);
    points->SetPoints(pts);

    // Create spline
    spline->SetPoints(pts);

    return true;

  }

  void ToolPathPlanner::smoothData(vtkSmartPointer<vtkParametricSpline> spline, vtkSmartPointer<vtkPolyData>& points, vtkSmartPointer<vtkPolyData>& derivatives)
  {


    vtkSmartPointer<vtkPoints> new_points = vtkSmartPointer<vtkPoints>::New();

    vtkSmartPointer<vtkDoubleArray> derv = vtkSmartPointer<vtkDoubleArray>::New();
    derv->SetNumberOfComponents(3);

    double new_pt[3] = {0, 0, 0};
    double* new_ptr;
    new_ptr = &new_pt[0];

    // get points which are evenly spaced along the spline
    // initialize num_line_pts to some number, find the Euclidean distance between two points (m & n),
    // then use this distance to determine how many points should be used in the given line
    int num_line_pts = 10;
    double m[3], n[3];

    double u[3], pt[3], d[9]; // u - search point, pt - resulting point, d - unused but still required
    u[0] = u[1] = u[2] = 0;

    // spline->Evaluate() takes a number in the range [0,1]
    spline->Evaluate(u, m, d);
    u[0] = 1/double(num_line_pts);
    spline->Evaluate(u, n, d);

    // calculate new point spacing
    double s = sqrt(pt_dist(&m[0], &n[0]));
    num_line_pts = round( double(num_line_pts) / (tool_.pt_spacing / s));

    // Get points evenly spaced along the spline
    for( int i = 0; i <= num_line_pts; ++i)
    {
      // Get point and store
      u[0] = double(i)/double(num_line_pts);
      u[1] = double(i)/double(num_line_pts);
      u[2] = double(i)/double(num_line_pts);
      spline->Evaluate(u, pt, d);
      new_points->InsertNextPoint(pt);

      double pt1[3], pt2[3];
      // find nearby points in order to calculate the local deriviative
      if(i == 0)
      {
        pt1[0] = pt[0];
        pt1[1] = pt[1];
        pt1[2] = pt[2];
        u[0] += 1/(1*double(num_line_pts));
        spline->Evaluate(u, pt2, d);
      }
      else if(i == num_line_pts)
      {
        pt2[0] = pt[0];
        pt2[1] = pt[1];
        pt2[2] = pt[2];
        u[0] -= 1/(1*double(num_line_pts));
        spline->Evaluate(u, pt1, d);
      }
      else
      {
        double u2[3] = {u[0], u[1], u[2]};
        u2[0] += 1/(1*double(num_line_pts));
        spline->Evaluate(u2, pt2, d);

        u[0] -= 1/(1*double(num_line_pts));
        spline->Evaluate(u, pt1, d);
      }

      // calculate the derivative
      new_ptr[0] = pt1[0] - pt2[0];
      new_ptr[1] = pt1[1] - pt2[1];
      new_ptr[2] = pt1[2] - pt2[2];
      double denom = sqrt(pow(new_ptr[0],2) + pow(new_ptr[1],2) + pow(new_ptr[2],2));

      // normalize the derivative vector
      new_ptr[0] /= denom;
      new_ptr[1] /= denom;
      new_ptr[2] /= denom;

      derv->InsertNextTuple(new_ptr);
    }
    // Set points and normals
    points->SetPoints(new_points);
    estimateNewNormals(points);

    // Set points and derivatives
    derivatives->SetPoints(new_points);
    derivatives->GetPointData()->SetNormals(derv);
  }

  // Sort points in linear order
  void ToolPathPlanner::sortPoints(vtkSmartPointer<vtkPoints>& points)
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
        if (calc_distance(sorted_points.back(), new_points[next]) < calc_distance(sorted_points.front(), new_points[next]))
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

    points = line_points;
  }

  void ToolPathPlanner::generateNormals(vtkSmartPointer<vtkPolyData>& data)
  {
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
    normalGenerator->SetFlipNormals(0);
    normalGenerator->SetNonManifoldTraversal(1);

    normalGenerator->Update();

    vtkDataArray* normals = normalGenerator->GetOutput()->GetPointData()->GetNormals();
    if(normals)
    {
      data->GetPointData()->SetNormals(normals);
    }
  }

  void ToolPathPlanner::estimateNewNormals(vtkSmartPointer<vtkPolyData>& data)
  {
    //Create the tree
    vtkSmartPointer<vtkKdTreePointLocator> pointTree =
        vtkSmartPointer<vtkKdTreePointLocator>::New();
    pointTree->SetDataSet(input_mesh_);
    pointTree->BuildLocator();

    // Find k nearest neighbors and use their normals to estimate the normal of the desired point
    vtkSmartPointer<vtkDoubleArray> new_norm = vtkSmartPointer<vtkDoubleArray>::New();
    new_norm->SetNumberOfComponents(3);

    for(int i = 0; i < data->GetPoints()->GetNumberOfPoints(); ++i)
    {
      // Get the next point to estimate the normal
      double* pt = data->GetPoints()->GetPoint(i);
      double testPoint[3] = {pt[0], pt[1], pt[2]};
      vtkSmartPointer<vtkIdList> result = vtkSmartPointer<vtkIdList>::New();

      // Find N nearest neighbors to the point
      pointTree->FindClosestNPoints(tool_.nearest_neighbors, testPoint, result);
      double final[3] = {0, 0, 0};
      int count = 0;

      // For N neighbors found, get their normals and add them together
      for(int j = 0; j < result->GetNumberOfIds(); ++j)
      {
        pt = input_mesh_->GetPointData()->GetNormals()->GetTuple(result->GetId(j));
        if(isnan(pt[0]) || isnan(pt[1]) || isnan(pt[2]))
        {
          continue;
        }
        final[0] += pt[0];
        final[1] += pt[1];
        final[2] += pt[2];
        ++count;
      }

      // If at least 1 neighbor is found, average the normals, else set to zero
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
      // Insert the averaged normal
      new_norm->InsertNextTuple(pt2);
    }
    // Insert the normal data
    data->GetPointData()->SetNormals(new_norm);
  }

  vtkSmartPointer<vtkPolyData> ToolPathPlanner::createOffsetLine(vtkSmartPointer<vtkPolyData> line, vtkSmartPointer<vtkPolyData> derivatives, double dist)
  {
    vtkSmartPointer<vtkPolyData> new_points;

    vtkDataArray* normals = line->GetPointData()->GetNormals();
    vtkDataArray* ders = derivatives->GetPointData()->GetNormals();

    // If normal or derivative data does not exist, or number of normals and derivatives do not match, return a null pointer
    if(!normals || !ders || normals->GetNumberOfTuples() != ders->GetNumberOfTuples())
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
    new_pts->InsertNextPoint(new_pt);

    // Extend front point
    pt1 = new_pts->GetPoint(0);
    new_pt[0] = pt1[0];
    new_pt[1] = pt1[1];
    new_pt[2] = pt1[2];
    pt1 = new_pts->GetPoint(1);
    new_pt[0] +=  (new_pt[0] - pt1[0]);
    new_pt[1] +=  (new_pt[1] - pt1[1]);
    new_pt[2] +=  (new_pt[2] - pt1[2]);
    new_pts->InsertNextPoint(new_pt);

    // because extended points were added to the back, need to resort the list
    sortPoints(new_pts);

    new_points->SetPoints(new_pts);

    estimateNewNormals(new_points);

    return new_points;
  }

  vtkSmartPointer<vtkPolyData> ToolPathPlanner::createSurfaceFromSpline(vtkSmartPointer<vtkPolyData> line, double dist)
  {
    vtkSmartPointer<vtkPolyData> new_surface;
    vtkSmartPointer<vtkDataArray> normals = line->GetPointData()->GetNormals();

    if(!normals)
    {
      return new_surface;
    }

    new_surface = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

    // for each point, insert 2 points, one above and one below, to create a new surface
    for(int i = 0; i < normals->GetNumberOfTuples(); ++i)
    {
      double* norm = normals->GetTuple(i);
      double* pt = line->GetPoints()->GetPoint(i);

      // insert the cell ids to create triangles
      if(i < normals->GetNumberOfTuples()-1)
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

      double new_pt[3];
      new_pt[0] = pt[0] + norm[0] * dist;
      new_pt[1] = pt[1] + norm[1] * dist;
      new_pt[2] = pt[2] + norm[2] * dist;

      points->InsertNextPoint( new_pt);

      new_pt[0] = pt[0] - norm[0] * dist;
      new_pt[1] = pt[1] - norm[1] * dist;
      new_pt[2] = pt[2] - norm[2] * dist;

      points->InsertNextPoint( new_pt);
    }

    new_surface->SetPolys(cells);
    new_surface->SetPoints(points);
    return new_surface;
  }

  void flipPointOrder(ProcessPath& path)
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

    // flip derivative directions
    vtkDataArray* ders = path.derivatives->GetPointData()->GetNormals();
    for(int i = 0; i < ders->GetNumberOfTuples(); ++i)
    {
      double* pt = ders->GetTuple(i);
      pt[0] *= -1;
      pt[1] *= -1;
      pt[2] *= -1;
      ders->SetTuple(i, pt);
    }
    path.derivatives->GetPointData()->SetNormals(ders);

    // reset points in spline
    path.spline->SetPoints(points);
  }



}
