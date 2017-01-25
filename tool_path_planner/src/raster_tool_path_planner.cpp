/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <limits>
#include <cmath>

#include <Eigen/Core>

#include <vtkParametricFunctionSource.h>
#include <vtkOBBTree.h>
#include <vtkIntersectionPolyDataFilter.h>
#include <vtkDelaunay2D.h>
#include <vtkMath.h>
#include <vtkSpline.h>
#include <vtkPolyDataNormals.h>
#include <vtkKdTreePointLocator.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkTriangle.h>
#include <vtk_viewer/vtk_utils.h>
#include <vtkReverseSense.h>
#include <vtkImplicitDataSet.h>
#include <vtkCutter.h>
#include <vtkCellLocator.h>
#include <vtkGenericCell.h>
#include <vtkTriangleFilter.h>

#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <tool_path_planner/raster_tool_path_planner.h>

namespace tool_path_planner
{

  RasterToolPathPlanner::RasterToolPathPlanner()
  {
    debug_on_ = false;
  }

  void RasterToolPathPlanner::planPaths(const vtkSmartPointer<vtkPolyData> mesh, std::vector<ProcessPath>& paths)
  {
    setInputMesh(mesh);
    input_mesh_->BuildLinks();
    input_mesh_->BuildCells();
    computePaths();
    paths = getPaths();
  }

  void RasterToolPathPlanner::planPaths(const std::vector<vtkSmartPointer<vtkPolyData> > meshes, std::vector< std::vector<ProcessPath> >& paths)
  {
    for(int i = 0; i < meshes.size(); ++i)
    {
      std::vector<ProcessPath> new_path;
      planPaths(meshes[i], new_path);
      paths.push_back(new_path);
    }
  }

  void RasterToolPathPlanner::planPaths(const std::vector<pcl::PolygonMesh>& meshes, std::vector< std::vector<ProcessPath> >& paths)
  {
    for(int i = 0; i < meshes.size(); ++i)
    {
      std::vector<ProcessPath> new_path;
      planPaths(meshes[i], new_path);
      paths.push_back(new_path);
    }
  }

  void RasterToolPathPlanner::planPaths(const pcl::PolygonMesh& mesh, std::vector<ProcessPath>& paths)
  {
    vtkSmartPointer<vtkPolyData> vtk_mesh;
    pcl::VTKUtils::mesh2vtk(mesh, vtk_mesh);
    planPaths(vtk_mesh, paths);
  }

  void RasterToolPathPlanner::setInputMesh(vtkSmartPointer<vtkPolyData> mesh)
  {
    if(!input_mesh_)
    {
      input_mesh_ = vtkSmartPointer<vtkPolyData>::New();
    }
    input_mesh_->DeepCopy(mesh);

    if(!kd_tree_)
    {
      kd_tree_ = vtkSmartPointer<vtkKdTreePointLocator>::New();
    }
    kd_tree_->SetDataSet(input_mesh_);
    kd_tree_->BuildLocator();

    // Add display for debugging
    debug_viewer_.removeAllDisplays();
    std::vector<float> color(3);
    color[0] = 0.9; color[1] = 0.9; color[2] = 0.9;
    debug_viewer_.addPolyDataDisplay(input_mesh_, color);

    // TODO: this does not appear to work
    vtk_viewer::generateNormals(input_mesh_);
  }

  bool RasterToolPathPlanner::computePaths()
  {
    // Need to call getFirstPath or other method to generate the first path
    // If no paths exist, there is nothing to create offset paths from
    if(paths_.size() != 1)
    {
      ProcessPath first_path;
      if(!getFirstPath(first_path))
      {
        return false;
      }
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

    // clear all but the first (mesh) display
    if(debug_on_)
    {
      int num_obj = debug_viewer_.getNumberOfDisplayObjects() - 1;
      for(int i = 0; i < num_obj; ++i)
      debug_viewer_.removeObjectDisplay(debug_viewer_.getNumberOfDisplayObjects() - 1);
    }


    std::vector<ProcessPath> new_paths;
    std::vector<int> delete_paths;
    for(int i = 0; i < paths_.size(); ++i)
    {
      std::vector<ProcessPath> out_paths;
      if(checkPathForHoles(paths_[i], out_paths))
      {
        // mark a path to delete
        delete_paths.push_back(i);
        for(int j = 0; j < out_paths.size(); ++j)
        {
          new_paths.push_back(out_paths[j]); // save all new paths
        }
      }
    }

    // if paths need to be modified, delete all old paths (starting at the back) and insert new ones
    for(int i = delete_paths.size() - 1; i >=0 ; --i )
    {
      paths_.erase(paths_.begin() + delete_paths[i]);
    }

    for(int i = 0; i < new_paths.size(); ++i)
    {
      paths_.push_back(new_paths[i]);

      if(debug_on_)  // cutting mesh display
      {
        std::vector<float> color(3);
        color[0] = 0.8; color[1] = 0.8; color[2] = 0.8;

        debug_viewer_.addPolyDataDisplay(new_paths[i].intersection_plane, color);
        debug_viewer_.renderDisplay();
      }
    }

    return true;
  }

  bool RasterToolPathPlanner::getFirstPath(ProcessPath& path)
  {
    // clear old paths before creating new
    paths_.clear();

    // generate first path according to algorithm in createStartCurve()
    vtkSmartPointer<vtkPolyData> start_curve = vtkSmartPointer<vtkPolyData>::New();
    start_curve = createStartCurve();

    // get input mesh bounds
    double bounds[6];
    input_mesh_->GetBounds(bounds);
    double x = fabs(bounds[1] - bounds[0]);
    double y = fabs(bounds[3] - bounds[2]);
    double z = fabs(bounds[5] - bounds[4]);

    // Use the bounds to determine how large to make the cutting mesh
    double max = x > y ? x : y;
    max = max > z ? max : z;
    vtkSmartPointer<vtkPolyData> cutting_mesh = createSurfaceFromSpline(start_curve, max);

    if(debug_on_)  // cutting mesh display
    {
      std::vector<float> color(3);
      color[0] = 0.8; color[1] = 0.8; color[2] = 0.8;

      debug_viewer_.addPolyDataDisplay(cutting_mesh, color);
      debug_viewer_.renderDisplay();
      debug_viewer_.removeObjectDisplay(debug_viewer_.getNumberOfDisplayObjects() - 1);
    }

    // use cutting mesh to find intersection line
    vtkSmartPointer<vtkPolyData> intersection_line = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();

    if(!findIntersectionLine(cutting_mesh, intersection_line, spline))
    {
      return false;
    }

    ProcessPath this_path;
    estimateNewNormals(intersection_line);
    this_path.intersection_plane = intersection_line;

    if(getNextPath(this_path, path, 0.0))
    {
      paths_.push_back(path);
      return true;
    }
    return false;
  }

  bool RasterToolPathPlanner::getNextPath(const ProcessPath this_path, ProcessPath& next_path, double dist)
  {
    if(dist == 0.0 && this_path.intersection_plane->GetPoints()->GetNumberOfPoints() < 2)
    {
      cout << "No path offset and no intersection plane given. Cannot generate next path\n";
      return false;
    }

    vtkSmartPointer<vtkPolyData> offset_line = vtkSmartPointer<vtkPolyData>::New();
    if(dist != 0.0)  // if offset distance given, create an offset surface
    {
      // create offset points
      offset_line = createOffsetLine(this_path.line, this_path.derivatives, dist);

      // create cutting surface  TODO: offset may need to be based upon point bounds
      next_path.intersection_plane = createSurfaceFromSpline(offset_line, tool_.intersecting_plane_height);
    }
    else  // if no offset distance given, use points in this_path.intersection_plane to create the surface
    {
      // resample points to make sure there the intersection filter works properly
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
      points = this_path.intersection_plane->GetPoints();
      resamplePoints(points);
      offset_line->SetPoints(points);
      estimateNewNormals(offset_line);

      next_path.intersection_plane = createSurfaceFromSpline(offset_line, tool_.intersecting_plane_height);
    }

    if(debug_on_)  // offset points and cutting mesh display
    {
      std::vector<float> color(3);
      color[0] = 0.9; color[1] = 0.2; color[2] = 0.2;
      debug_viewer_.addPolyNormalsDisplay(offset_line, color, tool_.pt_spacing);

      color[0] = 0.8; color[1] = 0.8; color[2] = 0.8;
      debug_viewer_.addPolyDataDisplay(next_path.intersection_plane, color);
      debug_viewer_.renderDisplay();
      debug_viewer_.removeObjectDisplay(debug_viewer_.getNumberOfDisplayObjects() - 2);
    }

    // use surface to find intersection line
    vtkSmartPointer<vtkPolyData> intersection_line = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();

    if(!findIntersectionLine(next_path.intersection_plane, intersection_line, spline))
    {
      return false;
    }

    if(debug_on_)  // spline display
    {
      std::vector<float> color(3);
      color[0] = 0.2; color[1] = 0.9; color[2] = 0.9;
      vtkSmartPointer<vtkParametricFunctionSource> source = vtkSmartPointer<vtkParametricFunctionSource>::New();
      source->SetParametricFunction(spline);
      source->Update();

      debug_viewer_.addPolyDataDisplay(source->GetOutput(), color);
      debug_viewer_.renderDisplay();
      debug_viewer_.removeObjectDisplay(debug_viewer_.getNumberOfDisplayObjects() - 1);
    }

    if(intersection_line->GetPoints()->GetNumberOfPoints() < 2)
    {
      return false;
    }

    //use spline to create interpolated data with normals and derivatives
    vtkSmartPointer<vtkPolyData> points = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> derivatives = vtkSmartPointer<vtkPolyData>::New();
    smoothData(spline, points, derivatives);

    if(debug_on_)  // points display
    {
      std::vector<float> color(3);
      color[0] = 0.2; color[1] = 0.2; color[2] = 0.9;
      debug_viewer_.addPolyNormalsDisplay(points, color, tool_.pt_spacing);
      color[0] = 0.9; color[1] = 0.9; color[2] = 0.2;
      debug_viewer_.addPolyNormalsDisplay(derivatives, color, tool_.pt_spacing);
      debug_viewer_.renderDisplay();
      debug_viewer_.removeObjectDisplay(debug_viewer_.getNumberOfDisplayObjects() - 1);
    }

    if(points->GetPoints()->GetNumberOfPoints() < 2)
    {
      return false;
    }
    next_path.line = points;
    next_path.spline = spline;
    next_path.derivatives = derivatives;

    // compare start/end points of new line to old line, flip order if necessary
    if(dist != 0.0)  // only flip if offset is non-zero (new_path)
    {
      int length = next_path.line->GetPoints()->GetNumberOfPoints();
      if(vtk_viewer::pt_dist(this_path.line->GetPoints()->GetPoint(0), next_path.line->GetPoints()->GetPoint(0))
         > vtk_viewer::pt_dist(this_path.line->GetPoints()->GetPoint(0), next_path.line->GetPoints()->GetPoint(length-1)))
      {
        flipPointOrder(next_path);
      }
    }

    if(debug_on_)  // points display
    {
      std::vector<float> color(3);
      color[0] = 0.9; color[1] = 0.9; color[2] = 0.2;
      debug_viewer_.addPolyNormalsDisplay(derivatives, color, tool_.pt_spacing);
      debug_viewer_.renderDisplay();
    }

    return true;
  }

  bool RasterToolPathPlanner::checkPathForHoles(const ProcessPath path, std::vector<ProcessPath>& out_paths)
  {
    // use cutting mesh to find intersection line
    vtkSmartPointer<vtkPolyData> intersection_line = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();

    if(!findIntersectionLine(path.intersection_plane, intersection_line, spline))
    {
      return false;
    }

    // create cell and triangle filters
    vtkSmartPointer<vtkCellLocator> locator = vtkSmartPointer<vtkCellLocator>::New();
    locator->SetDataSet(input_mesh_);
    locator->BuildLocator();

    vtkSmartPointer<vtkTriangleFilter> tri_filter = vtkSmartPointer<vtkTriangleFilter>::New();
    tri_filter->SetInputData(input_mesh_);
    tri_filter->Update();

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
      cell1 = locator->FindCell(pt1, tol, cell, pcoords, &weights[0]);
      if(cell1 != -1)
      {
        cell2 = locator->FindCell(pt2, tol, cell, pcoords, &weights[0]);
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
        if(dist > tool_.min_hole_size)
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
          resamplePoints(new_points);
          new_line->SetPoints(new_points);

          ProcessPath this_path, new_path;
          estimateNewNormals(new_line);
          this_path.intersection_plane = new_line;

          if(getNextPath(this_path, new_path, 0.0))
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
      estimateNewNormals(new_line);
      this_path.intersection_plane = new_line;

      if(getNextPath(this_path, new_path, 0.0))
      {
        out_paths.push_back(new_path);
      }
    }

    return out_paths.size() > 1;

  }

  vtkSmartPointer<vtkPolyData> RasterToolPathPlanner::createStartCurve()
  {
    // Find weighted center point and normal average of the input mesh
    vtkSmartPointer<vtkCellArray> cell_ids = input_mesh_->GetPolys();
    cell_ids->InitTraversal();

    double avg_center[3] = {0,0,0};
    double avg_norm[3] = {0,0,0};
    double avg_area = 0;
    int num_cells = cell_ids->GetNumberOfCells();

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

    vtkSmartPointer<vtkOBBTree> obb_tree = vtkSmartPointer<vtkOBBTree>::New();
    obb_tree->SetTolerance(0.001);
    obb_tree->SetLazyEvaluation(0);
    obb_tree->ComputeOBB(input_mesh_->GetPoints(), corner, max, mid, min, size);

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
      max[0] /= m;
      max[1] /= m;
      max[2] /= m;
      m = sqrt(mid[0] * mid[0] + mid[1] * mid[1] + mid[2] * mid[2]);
      mid[0] /= m;
      mid[1] /= m;
      mid[2] /= m;

      max[0] = (max[0] + mid[0]) * 5.0;
      max[1] = (max[1] + mid[1]) * 5.0;
      max[2] = (max[2] + mid[2]) * 5.0;
    }

    // Use the max axis to create additional points for the starting curve
    double pt[3];
    pt[0] = avg_center[0] + max[0];
    pt[1] = avg_center[1] + max[1];
    pt[2] = avg_center[2] + max[2];
    line->GetPoints()->InsertNextPoint(pt);

    line->GetPoints()->InsertNextPoint(avg_center);

    pt[0] = avg_center[0] - max[0];
    pt[1] = avg_center[1] - max[1];
    pt[2] = avg_center[2] - max[2];
    line->GetPoints()->InsertNextPoint(pt);

    // set the normal for all points inserted
    vtkSmartPointer<vtkDoubleArray> norms = vtkSmartPointer<vtkDoubleArray>::New();
    norms->SetNumberOfComponents(3);
    for(int i = 0; i < line->GetPoints()->GetNumberOfPoints(); ++i)
    {
      double n[3] ={avg_norm[0], avg_norm[1], avg_norm[2]};
      norms->InsertNextTuple(n);
    }
    line->GetPointData()->SetNormals(norms);

    return line;
  }

  bool RasterToolPathPlanner::getCellCentroidData(int id, double* center, double* norm, double& area)
  {

      vtkCell* cell = input_mesh_->GetCell(id);
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

        double* n = input_mesh_->GetCellData()->GetNormals()->GetTuple(id);
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

  bool RasterToolPathPlanner::findIntersectionLine(vtkSmartPointer<vtkPolyData> cut_surface,
                                         vtkSmartPointer<vtkPolyData>& points,
                                         vtkSmartPointer<vtkParametricSpline>& spline)
  {
    // Find the intersection between the input mesh and given cutting surface
    vtkSmartPointer<vtkIntersectionPolyDataFilter> intersection_filter =
      vtkSmartPointer<vtkIntersectionPolyDataFilter>::New();
    intersection_filter->SetInputData( 0, input_mesh_);
    intersection_filter->SetInputData( 1, cut_surface );

    if(cut_surface->GetNumberOfCells() < 1)
    {
      return false;
    }

    try
    {
      intersection_filter->Update();
    }
    catch(...)
    {
      cout << "error in intersection filter\n";
      return false;
    }

    // if no intersection found, return false
    if(intersection_filter->GetStatus() == 0)
    {
      return false;
    }

    vtkSmartPointer<vtkPolyData> output = intersection_filter->GetOutput();
    if(!output)
    {
      return false;
    }

    // Check the number of points in the intersection, if not enough to process, return false
    vtkSmartPointer<vtkPoints> pts = intersection_filter->GetOutput()->GetPoints();
    if(!pts || pts->GetNumberOfPoints() <= 2)
    {
      return false;
    }

    // Intersection points are not ordered; order them so that they form a continous smooth line
    sortPoints(pts);
    points->SetPoints(pts);

    // Create spline from ordered points
    spline->SetPoints(pts);

    return true;

  }

  void RasterToolPathPlanner::resamplePoints(vtkSmartPointer<vtkPoints>& points)
  {
    vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
    spline->SetPoints(points);

    int num_line_pts = points->GetNumberOfPoints()/2.0;

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

  void RasterToolPathPlanner::smoothData(vtkSmartPointer<vtkParametricSpline> spline, vtkSmartPointer<vtkPolyData>& points, vtkSmartPointer<vtkPolyData>& derivatives)
  {
    vtkSmartPointer<vtkPoints> new_points = vtkSmartPointer<vtkPoints>::New();

    vtkSmartPointer<vtkDoubleArray> derv = vtkSmartPointer<vtkDoubleArray>::New();
    derv->SetNumberOfComponents(3);

    Eigen::Vector3d new_pt(0, 0, 0);

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
    double s = sqrt(vtk_viewer::pt_dist(&m[0], &n[0]));
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

      // calculate the derivative and normalize
      new_pt[0] = pt1[0] - pt2[0];
      new_pt[1] = pt1[1] - pt2[1];
      new_pt[2] = pt1[2] - pt2[2];
      new_pt.normalize();

      // insert the next derivative
      double new_ptr[3] = {new_pt[0], new_pt[1], new_pt[2]};
      derv->InsertNextTuple(&new_ptr[0]);
    }
    // Set points and normals
    points->SetPoints(new_points);
    estimateNewNormals(points);

    // Set points and derivatives
    derivatives->SetPoints(new_points);
    derivatives->GetPointData()->SetNormals(derv);
  }

  // Sort points in linear order
  void RasterToolPathPlanner::sortPoints(vtkSmartPointer<vtkPoints>& points)
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
        if (squared_distance(sorted_points.back(), new_points[next]) < squared_distance(sorted_points.front(), new_points[next]))
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

  void RasterToolPathPlanner::generateNormals(vtkSmartPointer<vtkPolyData>& data)
  {
    vtkSmartPointer<vtkPolyDataNormals> normal_generator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normal_generator->SetInputData(data);
    normal_generator->ComputePointNormalsOn();
    normal_generator->ComputeCellNormalsOn();

    // Optional settings
    normal_generator->SetFeatureAngle(0.1);
    normal_generator->SetSplitting(0);
    normal_generator->SetConsistency(1);
    normal_generator->SetAutoOrientNormals(1);
    normal_generator->SetComputePointNormals(1);
    normal_generator->SetComputeCellNormals(1);
    normal_generator->SetFlipNormals(0);
    normal_generator->SetNonManifoldTraversal(1);

    normal_generator->Update();

    vtkDataArray* normals = normal_generator->GetOutput()->GetPointData()->GetNormals();
    if(normals)
    {
      data->GetPointData()->SetNormals(normals);
    }
  }

  void RasterToolPathPlanner::estimateNewNormals(vtkSmartPointer<vtkPolyData>& data)
  {
    // Find k nearest neighbors and use their normals to estimate the normal of the desired point
    vtkSmartPointer<vtkDoubleArray> new_norm = vtkSmartPointer<vtkDoubleArray>::New();
    new_norm->SetNumberOfComponents(3);
    new_norm->SetNumberOfTuples(data->GetPoints()->GetNumberOfPoints());

    for(int i = 0; i < data->GetPoints()->GetNumberOfPoints(); ++i)
    {
      // Get the next point to estimate the normal
      double pt[3];
      data->GetPoints()->GetPoint(i, &pt[0]);


      double testPoint[3] = {pt[0], pt[1], pt[2]};
      vtkSmartPointer<vtkIdList> result = vtkSmartPointer<vtkIdList>::New();

      // Find N nearest neighbors to the point, if N is greater than # of points, return all points
      if(tool_.nearest_neighbors > input_mesh_->GetPoints()->GetNumberOfPoints())
      {
        for(int j = 0; j < input_mesh_->GetPoints()->GetNumberOfPoints(); ++j)
        {
          result->InsertNextId(j);
        }
      }
      else
      {
        kd_tree_->FindClosestNPoints(tool_.nearest_neighbors, testPoint, result);
      }
      Eigen::Vector3d final(0, 0, 0);
      int count = 0;

      // For N neighbors found, get their normals and add them together
      for(int j = 0; j < result->GetNumberOfIds(); ++j)
      {
        double pt2[3];
        input_mesh_->GetPointData()->GetNormals()->GetTuple(result->GetId(j), &pt2[0]);
        if(std::isnan(pt2[0]) || std::isnan(pt2[1]) || std::isnan(pt2[2]))
        {
          continue;
        }
        final[0] += pt2[0];
        final[1] += pt2[1];
        final[2] += pt2[2];
        ++count;
      }

      // If at least 1 neighbor is found, average the normals, else set to zero
      if(count > 0)
      {
        final[0] /= double(count);
        final[1] /= double(count);
        final[2] /= double(count);
        final.normalize();
      }

      // Insert the averaged normal
      new_norm->SetTuple3(i, final[0], final[1], final[2]);
    }
    // Insert the normal data
    data->GetPointData()->SetNormals(new_norm);
  }

  vtkSmartPointer<vtkPolyData> RasterToolPathPlanner::createOffsetLine(vtkSmartPointer<vtkPolyData> line, vtkSmartPointer<vtkPolyData> derivatives, double dist)
  {
    vtkSmartPointer<vtkPolyData> new_points;

    vtkDataArray* normals = line->GetPointData()->GetNormals();
    vtkDataArray* ders = derivatives->GetPointData()->GetNormals();

    // If normal or derivative data does not exist, or number of normals and derivatives do not match, return a null pointer
    if(!normals || !ders || normals->GetNumberOfTuples() != ders->GetNumberOfTuples())
    {
      return new_points;
    }

    if(normals->GetNumberOfTuples() != line->GetNumberOfPoints())
    {
      cout << "ERROR IN CALC OFFSET LINE\n";
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
      w.normalize();

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

  vtkSmartPointer<vtkPolyData> RasterToolPathPlanner::createSurfaceFromSpline(vtkSmartPointer<vtkPolyData> line, double dist)
  {
    vtkSmartPointer<vtkPolyData> new_surface;
    vtkSmartPointer<vtkDataArray> normals = line->GetPointData()->GetNormals();

    if(!normals)
    {
      cout << "no normals, cannot create surface\n";
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

}
