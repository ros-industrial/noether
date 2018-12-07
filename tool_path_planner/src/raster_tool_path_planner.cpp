/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <limits>
#include <cmath>
#include <console_bridge/console.h>

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
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <tool_path_planner/raster_tool_path_planner.h>


static const std::size_t MAX_ATTEMPTS = 1000;

/**
 * @brief computes the angle between two vectors
 * @param v1
 * @param v2
 * @return The angle in radians
 */
double computeAngle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  return std::acos(v1.dot(v2)/(v1.norm() * v2.norm()));
}

namespace tool_path_planner
{

  RasterToolPathPlanner::RasterToolPathPlanner(bool use_ransac):
      use_ransac_normal_estimation_(use_ransac)
  {
    debug_on_ = false;
    cut_direction_[0] = cut_direction_[1] = cut_direction_[2] = 0;
    cut_centroid_[0] = cut_centroid_[1] = cut_centroid_[2] = 0;
    console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
  }

  void RasterToolPathPlanner::planPaths(const vtkSmartPointer<vtkPolyData> mesh, std::vector<ProcessPath>& paths)
  {
    setInputMesh(mesh);
    paths_.clear();
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

    // initializing search trees
    kd_tree_ = vtkSmartPointer<vtkKdTreePointLocator>::New();
    kd_tree_->SetDataSet(input_mesh_);
    kd_tree_->BuildLocator();
    cell_locator_ = vtkSmartPointer<vtkCellLocator>::New();
    cell_locator_->SetDataSet(input_mesh_);
    cell_locator_->BuildLocator();
    bsp_tree_ = vtkSmartPointer<vtkModifiedBSPTree>::New();
    bsp_tree_->SetDataSet(input_mesh_);
    bsp_tree_->BuildLocator();

    // Add display for debugging
    if(debug_on_)
    {
      debug_viewer_.removeAllDisplays();
      std::vector<float> color(3);
      color[0] = 0.9; color[1] = 0.9; color[2] = 0.9;
      debug_viewer_.addPolyDataDisplay(input_mesh_, color);
    }

    // TODO: this does not appear to work
    vtk_viewer::generateNormals(input_mesh_,0);
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
    int max = MAX_ATTEMPTS;
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
    if(!delete_paths.empty())
    {
     std::cout<<"Deleting "<<delete_paths.size()<<" paths"<<std::endl;
    }
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

    //vtkSmartPointer<vtkPolyData> cutting_mesh = createSurfaceFromSpline(start_curve, max);
    vtkSmartPointer<vtkPolyData> cutting_mesh = extrudeSplineToSurface(start_curve, max);
    if(!cutting_mesh)
    {
      return false;
    }

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
      cout << "No intersection found\n";
      return false;
    }

    ProcessPath this_path;
    //estimateNewNormals(intersection_line);
    if(!computeSurfaceLineNormals(intersection_line))
    {
      return false;
    }
    this_path.intersection_plane = intersection_line;

    if(getNextPath(this_path, path, 0.0))
    {
      paths_.push_back(path);
      return true;
    }
    return false;
  }

  bool RasterToolPathPlanner::getNextPath(const ProcessPath this_path, ProcessPath& next_path, double dist, bool test_self_intersection)
  {
    if(dist == 0.0 && this_path.intersection_plane->GetPoints()->GetNumberOfPoints() < 2)
    {
      cout << "No path offset and no intersection plane given. Cannot generate next path\n";
      return false;
    }

    vtkSmartPointer<vtkPolyData> offset_line = vtkSmartPointer<vtkPolyData>::New();
    if(dist != 0.0)  // if offset distance given, create an offset surface
    {
      // create offset points in the adjacent line at a distance "dist"
      offset_line = createOffsetLine(this_path.line, this_path.derivatives, dist);
      if(!offset_line)
      {
        return false;
      }

      // create cutting surface  TODO: offset may need to be based upon point bounds
      next_path.intersection_plane = extrudeSplineToSurface(offset_line, tool_.intersecting_plane_height);
      if(!next_path.intersection_plane)
      {
        return false;
      }
    }
    else  // if no offset distance given, use points in this_path.intersection_plane to create the surface
    {
      // resample points to make sure there the intersection filter works properly
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
      points = this_path.intersection_plane->GetPoints();
      resamplePoints(points);
      offset_line->SetPoints(points);
      //estimateNewNormals(offset_line);
      if(!computeSurfaceLineNormals(offset_line))
      {
        return false;
      }

      next_path.intersection_plane = extrudeSplineToSurface(offset_line, tool_.intersecting_plane_height);
      if(!next_path.intersection_plane)
      {
        return false;
      }
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
      logDebug("No intersection found for creating spline");
      return false;
    }

    // Check for self intersection (intersection of next path with the last path computed
    // If self-intersection occurs, return false (done planning paths)
    if(test_self_intersection && paths_.size() >= 1)
    {
      vtkSmartPointer<vtkIntersectionPolyDataFilter> intersection_filter =
        vtkSmartPointer<vtkIntersectionPolyDataFilter>::New();
      intersection_filter->SetSplitFirstOutput(0);
      intersection_filter->SetSplitSecondOutput(0);
      intersection_filter->SetInputData( 0, next_path.intersection_plane);
      intersection_filter->SetInputData( 1, paths_.back().intersection_plane);
      intersection_filter->Update();
      if(intersection_filter->GetOutput()->GetPoints()->GetNumberOfPoints() > 0)
      {
        logDebug("Self intersection found with back path");
        return false;
      }

      intersection_filter->SetInputData( 1, paths_.front().intersection_plane);
      intersection_filter->Update();
      if(intersection_filter->GetOutput()->GetPoints()->GetNumberOfPoints() > 0)
      {
        logDebug("Self intersection found with front path");
        return false;
      }
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
      cout << "Number of points after smoothing is less than 2\n";
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
      cell1 = cell_locator_->FindCell(pt1, tol, cell, pcoords, &weights[0]);
      if(cell1 != -1)
      {
        cell2 = cell_locator_->FindCell(pt2, tol, cell, pcoords, &weights[0]);
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

          if(new_points->GetNumberOfPoints() >= 2)
          {
            resamplePoints(new_points);
          }
          else
          {
            logDebug("Hole segment has less than 2 points, skipping resampling");
          }

          new_line->SetPoints(new_points);

          ProcessPath this_path, new_path;
          //estimateNewNormals(new_line);
          computeSurfaceLineNormals(new_line);
          this_path.intersection_plane = new_line;

          if(getNextPath(this_path, new_path, 0.0, false))
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
      //estimateNewNormals(new_line);
      computeSurfaceLineNormals(new_line);
      this_path.intersection_plane = new_line;

      if(getNextPath(this_path, new_path, 0.0, false))
      {
        out_paths.push_back(new_path);
      }
    }

    return out_paths.size() >= 1;

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

    if(cut_direction_[0] || cut_direction_[1] || cut_direction_[2])
    {
      max[0] = cut_direction_[0]; max[1] = cut_direction_[1]; max[2] = cut_direction_[2];
      avg_center[0] = cut_centroid_[0]; avg_center[1] = cut_centroid_[1]; avg_center[2] = cut_centroid_[2];
    }
    else
    {
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
        double temp_max = m;
        max[0] /= m;
        max[1] /= m;
        max[2] /= m;
        m = sqrt(mid[0] * mid[0] + mid[1] * mid[1] + mid[2] * mid[2]);
        mid[0] /= m;
        mid[1] /= m;
        mid[2] /= m;

        max[0] = (max[0] + mid[0]) * temp_max;
        max[1] = (max[1] + mid[1]) * temp_max;
        max[2] = (max[2] + mid[2]) * temp_max;
      }
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
      Eigen::Vector3d m(avg_norm[0], avg_norm[1], avg_norm[2]);
      m.normalize();

      double n[3] ={m[0], m[1], m[2]};

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
    // Check to make sure that the input cut surface contains a valid mesh
    if(cut_surface->GetNumberOfCells() < 1)
    {
      cout << "Number of input cells for calculating intersection is less than 1.\n";
      cout << "Cannot compute intersection.\n";
      return false;
    }

    // Find the intersection between the input mesh and given cutting surface
    vtkSmartPointer<vtkIntersectionPolyDataFilter> intersection_filter =
      vtkSmartPointer<vtkIntersectionPolyDataFilter>::New();
    intersection_filter->SetSplitFirstOutput(0);
    intersection_filter->SetSplitSecondOutput(0);
    intersection_filter->SetInputData( 0, input_mesh_);
    intersection_filter->SetInputData( 1, cut_surface );
    intersection_filter->GlobalWarningDisplayOff();

    intersection_filter->Update();

    // Check output of intersection to see if it is valid
    vtkSmartPointer<vtkPolyData> output = intersection_filter->GetOutput();
    if(!output)
    {
      return false;
    }

    // if no intersection found, return false
    vtkSmartPointer<vtkPoints> pts = intersection_filter->GetOutput()->GetPoints();
    if(intersection_filter->GetStatus() == 0 || !pts || pts->GetNumberOfPoints() <= 1)
    {
      return false;
    }

    vtkSmartPointer<vtkPoints> temp_pts = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();

    // get the intersection filter output and find a continous line segment
    poly_data = intersection_filter->GetOutput();
    getConnectedIntersectionLine(poly_data, temp_pts);

    if(temp_pts->GetNumberOfPoints() == 0)
    {
      logError("No connected lines were found");
      return false;
    }

    // return points and spline
    points->SetPoints(temp_pts);
    spline->SetPoints(temp_pts);

    return true;

  }

  void RasterToolPathPlanner::getConnectedIntersectionLine(vtkSmartPointer<vtkPolyData> line,
                                                           vtkSmartPointer<vtkPoints>& points)
  {
    int start = 0;
    std::vector<int> used_ids;
    std::vector<vtkSmartPointer<vtkPoints> > lines;
    int num_points = line->GetNumberOfPoints();

    while(used_ids.size() < num_points)
    {
      for(int i = 0; i < num_points; ++i)
      {
        // if id 'i' is not found, use it as the next index to start with for finding connected lines
        if(std::find(used_ids.begin(), used_ids.end(), i) == used_ids.end())
        {
          start = i;
          break;
        }
      }
      vtkSmartPointer<vtkPoints> temp_points = vtkSmartPointer<vtkPoints>::New();

      // only add the line segment if it is large enough
      double dist = getConnectedIntersectionLine(line, temp_points, used_ids, start);
      if( dist > tool_.min_segment_size)
      {
        lines.push_back(temp_points);
      }
    }

    // Now that we have 1 or more lines, check to see if we need to merge/delete lines
    if(lines.size() == 1)
    {
      points = lines[0];  // if only one line, return it
    }
    else if(lines.size() > 1)
    {
      // check ends of each line for merging/deleting
      vtkSmartPointer<vtkPoints> temp_points = vtkSmartPointer<vtkPoints>::New();
      // find largest line to start with
      int size = 0;
      int index = 0;
      for(int i = 0; i < lines.size(); ++i)
      {
        if(lines[i]->GetNumberOfPoints() > size)
        {
          index = i;
          size = lines[i]->GetNumberOfPoints();
        }
      }
      temp_points = lines[index];
      lines.erase(lines.begin() + index);

      while(lines.size() > 0)
      {
        int next_index = 0;
        int order = 0;
        double min_dist = std::numeric_limits<float>::max();

        // find the next closest line
        for(int i = 0; i < lines.size(); ++i)
        {
          // get distance
          double dist1 = vtk_viewer::pt_dist(temp_points->GetPoint(0), lines[i]->GetPoint(0));
          double dist2 = vtk_viewer::pt_dist(temp_points->GetPoint( temp_points->GetNumberOfPoints() - 1 ),
                                             lines[i]->GetPoint(0) );
          double dist3 = vtk_viewer::pt_dist(temp_points->GetPoint(0),
                                             lines[i]->GetPoint(lines[i]->GetNumberOfPoints() - 1) );
          double dist4 = vtk_viewer::pt_dist(temp_points->GetPoint( temp_points->GetNumberOfPoints() - 1 ),
                                             lines[i]->GetPoint(lines[i]->GetNumberOfPoints() - 1) );


          double dist = std::min(dist1, dist2);
          dist = std::min(dist, dist3);
          dist = std::min(dist, dist4);

          // store distance and index if it is the smallest
          // also store the order, used for later concatenation (front->front, back->front, front->back, back->back)
          if(dist < min_dist)
          {
            next_index = i;
            min_dist = dist;

            // determining joining strategy, lg (final continous line "temp_points") and lc (current line
            order = (dist1 == min_dist) ? 1 : order;  // join lines starting points lc(begin) -> lg(begin)
            order = (dist2 == min_dist) ? 2 : order;  // join lg(end) -> lc(begin)
            order = (dist3 == min_dist) ? 3 : order;  // join lg(begin) -> lc(end)
            order = (dist4 == min_dist) ? 4 : order;  // join lg(end) -> lc(end)
          }
        }

        // Use new index to append lines together or delete lines
        // VTK does not support inserting new points at the front so sometimes we need to create a new object to insert points to
        switch (order)
        {
        case 1:
        {
          vtkSmartPointer<vtkPoints> tmp = vtkSmartPointer<vtkPoints>::New();

          for(int i = lines[next_index]->GetNumberOfPoints() - 1 ; i >=0 ; --i)
          {
            double pt[3];
            lines[next_index]->GetPoint(i, pt);
            tmp->InsertNextPoint(pt);
          }
          for(int i = 0; i < temp_points->GetNumberOfPoints(); ++i)
          {
            double pt[3];
            temp_points->GetPoint(i, pt);
            tmp->InsertNextPoint(pt);
          }
          temp_points->SetNumberOfPoints(tmp->GetNumberOfPoints());
          temp_points->DeepCopy(tmp);
          break;
        }
        case 2:
        {
          for(int i = 0; i < lines[next_index]->GetNumberOfPoints(); ++i)
          {
            temp_points->InsertNextPoint(lines[next_index]->GetPoint(i));
          }
          break;
        }
        case 3:
        {
          vtkSmartPointer<vtkPoints> tmp = vtkSmartPointer<vtkPoints>::New();

          for(int i = 0 ; i < lines[next_index]->GetNumberOfPoints(); ++i)
          {
            double pt[3];
            lines[next_index]->GetPoint(i, pt);
            tmp->InsertNextPoint(pt);
          }
          for(int i = 0; i < temp_points->GetNumberOfPoints(); ++i)
          {
            double pt[3];
            temp_points->GetPoint(i, pt);
            tmp->InsertNextPoint(pt);
          }
          temp_points->SetNumberOfPoints(tmp->GetNumberOfPoints());
          temp_points->DeepCopy(tmp);
          break;
        }
        case 4:
        {
          for(int i = lines[next_index]->GetNumberOfPoints() - 1 ; i >= 0; --i)
          {
            temp_points->InsertNextPoint(lines[next_index]->GetPoint(i));
          }
          break;
        }
        default:
          break;
        }
        lines.erase(lines.begin() + next_index);

      }// end of while loop
      points = temp_points;
    }// end concatenating lines together

  }

  double RasterToolPathPlanner::getConnectedIntersectionLine(vtkSmartPointer<vtkPolyData> line, vtkSmartPointer<vtkPoints>& points, std::vector<int>& used_ids, int start_pt)
  {
    int search_location = 0;
    int num_points = line->GetNumberOfPoints();
    int num_lines = line->GetNumberOfLines();

    vtkSmartPointer<vtkCellArray> line_data = vtkSmartPointer<vtkCellArray>::New();
    line_data = line->GetLines();
    line_data->InitTraversal();

    std::vector<int> ids;

    // get line segment for given point
    ids.push_back(start_pt);
    int next_id = start_pt;
    double line_length = 0.0;

    while(ids.size() < num_points)
    {
      // get the next point id, loop through all line to find
      int j = 0;

      // if next_id is already used, exit
      if(std::find(used_ids.begin(), used_ids.end(), next_id) != used_ids.end())
      {
        break;
      }

      line_data->InitTraversal();
      for(j = 0; j < num_lines; ++j)
      {
        vtkSmartPointer<vtkIdList> temp_cell = vtkSmartPointer<vtkIdList>::New();
        line_data->GetNextCell(temp_cell);
        if(temp_cell->GetNumberOfIds() == 0)
        {
          continue;
        }
        if(next_id == temp_cell->GetId(search_location))
        {
          // based on search direction, determine which value to insert in the id list (first or second)
          int location = (search_location + 1) % 2;
          double pt[3], pt_temp[3];
          line->GetPoint(next_id, pt);

          next_id = temp_cell->GetId(location);
          line->GetPoint(next_id, pt_temp);

          // get distance of current line segment and add to total distance
          line_length += sqrt(vtk_viewer::pt_dist(&pt[0], &pt_temp[0]));

          // insert next id
          if(search_location)
          {
            ids.insert(ids.begin(), next_id);
          }
          else
          {
            ids.push_back(next_id);
          }
          break;
        }
      }

      // If we have gone around a complete loop, next_id will equal first id, then break
      if( (next_id == ids.front() && search_location == 0) || (next_id == ids.back() && search_location == 1))
      {
        break;
      }

      // completed loop without finding a match
      if(j == num_lines && ids.size() < num_points && search_location == 0)
      {
          search_location = 1;
          next_id = ids[0];
        // search from the front to connect points
      }
      else if(j == num_lines && search_location == 1)
      {
        break;
      }
    } // end loop getting connected lines

    // copy the ids used into the in_id list
    used_ids.insert(used_ids.end(), ids.begin(), ids.end());

    // Copy the line ids into a VTK list
    vtkSmartPointer<vtkIdList> connected_pts = vtkSmartPointer<vtkIdList>::New();
    connected_pts->SetNumberOfIds(ids.size());
    for(int i = 0; i < ids.size(); ++i)
    {
      connected_pts->SetId(i, ids[i]);
    }

    // set point size and copy points to return
    points->Reset();
    points->Squeeze();
    points->SetNumberOfPoints(ids.size());
    line->GetPoints()->GetPoints(connected_pts, points);

    return line_length;
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

  void RasterToolPathPlanner::smoothData(vtkSmartPointer<vtkParametricSpline> spline, vtkSmartPointer<vtkPolyData>& points,
                                         vtkSmartPointer<vtkPolyData>& derivatives)
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
/*    spline->Evaluate(u, m, d);
    u[0] = 1/double(num_line_pts);
    spline->Evaluate(u, n, d);

    // calculate new point spacing
    double s = sqrt(vtk_viewer::pt_dist(&m[0], &n[0]));
    num_line_pts = round( double(num_line_pts) / (tool_.pt_spacing / s));*/

    // computing entire line length
    double spline_length = 0;
    std::array<double, 3> p1, p2;
    spline->GetPoints()->GetPoint(0,p1.data());
    for(std::size_t i = 1; i < spline->GetPoints()->GetNumberOfPoints(); i++)
    {
      spline->GetPoints()->GetPoint(i,p2.data());
      spline_length += sqrt(vtk_viewer::pt_dist(p1.data(), p2.data()));
      std::copy(p2.begin(),p2.end(),p1.begin());
    }

    num_line_pts = std::ceil(spline_length/tool_.pt_spacing) + 1;

    // Get points evenly spaced along the spline
    const double du = 1.0/(1.0*double(num_line_pts));
    for( int i = 0; i <= num_line_pts; ++i)
    {
      // Get point and store
      u[0] = i * du; // double(i)/double(num_line_pts);
      u[1] = i * du; // double(i)/double(num_line_pts);
      u[2] = i * du; // double(i)/double(num_line_pts);
      spline->Evaluate(u, pt, d);
      new_points->InsertNextPoint(pt);


      double pt1[3], pt2[3];
      // find nearby points in order to calculate the local derivative
      if(i == 0)
      {
        pt1[0] = pt[0];
        pt1[1] = pt[1];
        pt1[2] = pt[2];
        u[0] += du;
        spline->Evaluate(u, pt2, d);
      }
      else if(i == num_line_pts)
      {
        pt2[0] = pt[0];
        pt2[1] = pt[1];
        pt2[2] = pt[2];
        u[0] -= du;
        spline->Evaluate(u, pt1, d);
      }
      else
      {
        double u2[3] = {u[0], u[1], u[2]};
        u2[0] += du;
        spline->Evaluate(u2, pt2, d);

        u[0] -= du;
        spline->Evaluate(u, pt1, d);
      }

      // calculate the derivative and normalize
      new_pt[0] = pt1[0] - pt2[0];
      new_pt[1] = pt1[1] - pt2[1];
      new_pt[2] = pt1[2] - pt2[2];
      new_pt *= -1;
      new_pt.normalize();

      // insert the next derivative
      double new_ptr[3] = {new_pt[0], new_pt[1], new_pt[2]};
      derv->InsertNextTuple(&new_ptr[0]);
    }
    // Set points and normals
    points->SetPoints(new_points);
    //estimateNewNormals(points);
    computeSurfaceLineNormals(points);

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

  bool RasterToolPathPlanner::computeSurfaceLineNormals(vtkSmartPointer<vtkPolyData>& data)
  {
    // Find closest cell to each point and uses its normal vector
    vtkSmartPointer<vtkDoubleArray> new_norm = vtkSmartPointer<vtkDoubleArray>::New();
    new_norm->SetNumberOfComponents(3);
    new_norm->SetNumberOfTuples(data->GetPoints()->GetNumberOfPoints());

    for(int i = 0; i < data->GetPoints()->GetNumberOfPoints(); ++i)
    {

      // locate closest cell
      std::array<double,3> query_point;
      std::array<double,3> closest_point;
      vtkIdType cell_id;
      int sub_index;
      double dist;
      data->GetPoints()->GetPoint(i, query_point.data());
      cell_locator_->FindClosestPoint(query_point.data(),closest_point.data(),cell_id,sub_index,dist);
      if(cell_id < 0)
      {
        logError("FindClosestPoint returned an invalid cell id");
        return false;
      }

      // get normal
      Eigen::Vector3d normal_vect = Eigen::Vector3d::Zero();
      input_mesh_->GetCellData()->GetNormals()->GetTuple(cell_id,normal_vect.data());
      new_norm->SetTuple3(i,normal_vect(0),normal_vect(1),normal_vect(2));

    }
    // Insert the normal data
    data->GetPointData()->SetNormals(new_norm);
    return true;
  }

  void RasterToolPathPlanner::estimateNewNormals(vtkSmartPointer<vtkPolyData>& data)
  {
    if(use_ransac_normal_estimation_){
      estimateNewNormalsRansac(data);
      return;
    }

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
  void RasterToolPathPlanner::estimateNewNormalsRansac(vtkSmartPointer<vtkPolyData>& data)
  {
    // Find k nearest neighbors and use their normals to estimate the normal of the desired point
    vtkSmartPointer<vtkDoubleArray> new_norm = vtkSmartPointer<vtkDoubleArray>::New();
    new_norm->SetNumberOfComponents(3);
    new_norm->SetNumberOfTuples(data->GetPoints()->GetNumberOfPoints());

    for(int i = 0; i < data->GetPoints()->GetNumberOfPoints(); ++i)
    {
      // Get the next point to estimate the normal
      double pt[3],pn[3];
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
      input_mesh_->GetPointData()->GetNormals()->GetTuple(result->GetId(0), &pn[0]);

      Eigen::Vector3d final(0, 0, 0);

      pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      int cloud_count = 0;
      for(int j = 0; j < result->GetNumberOfIds(); ++j)
      {
        double pt2[3];
        input_mesh_->GetPoints()->GetPoint(result->GetId(j),&pt2[0]);
        if(std::isnan(pt2[0]) || std::isnan(pt2[1]) || std::isnan(pt2[2]))
        {
          continue;
        }
        local_cloud->push_back(pcl::PointXYZ(pt2[0],pt2[1],pt2[2]));
        cloud_count++;
      }
      std::vector<int> inliers;
      inliers.clear();
      pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
          model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (local_cloud));
      pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p,tool_.plane_fit_threhold);

      bool isgood = ransac.computeModel();
      Eigen::VectorXf icoeff;
      Eigen::VectorXf coeff;
      if(isgood){
        std::vector<int> inliers;
        ransac.getInliers(inliers);
        ransac.getModelCoefficients (icoeff);
        if(inliers.size()>3){
          model_p->optimizeModelCoefficients(inliers,icoeff,coeff);
        }
        else{
          coeff[0] = icoeff[0];
          coeff[1] = icoeff[1];
          coeff[2] = icoeff[2];
        }
        // switch sign of normal if its not aligned with original point's normal
        double dot = coeff[0]*pn[0] + coeff[1]*pn[1] + coeff[2]*pn[2];
        int normal_sign = 1;
        if(dot<0) normal_sign = -1;
        final[0] = normal_sign*coeff[0];
        final[1] = normal_sign*coeff[1];
        final[2] = normal_sign*coeff[2];
      }
      else{
        cout << "ransac failed to find a plane\n";
      }
      final.normalize(); // not sure we need this

      // Insert the ransac normal
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
      logError("Could not create offset line");
      return new_points;
    }

    if(normals->GetNumberOfTuples() != line->GetNumberOfPoints())
    {
      logError("ERROR IN CALC OFFSET LINE");
      return new_points;
    }

    new_points = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> new_pts = vtkSmartPointer<vtkPoints>::New();

    // calculate offset for each point
    Eigen::Vector3d offset_dir;
    for(int i = 0; i < normals->GetNumberOfTuples(); ++i)
    {
      //calculate cross to get offset direction
      double* nrml = normals->GetTuple(i);
      double* line_dir = ders->GetTuple(i);

      Eigen::Vector3d u(nrml[0], nrml[1], nrml[2]);
      Eigen::Vector3d v(line_dir[0], line_dir[1], line_dir[2]);
      Eigen::Vector3d w = u.cross(v);
      w.normalize();

      if(i == 0)
      {
        offset_dir = w;
      }
      else
      {
        // check offset direction consistency to correct due to flipped normal
        double angle = computeAngle(offset_dir,w);
        if(angle > M_PI_2)
        {
          // flip direction of w
          w *= -1;
        }

        offset_dir = w;
      }

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
    new_pts->SetPoint(new_pts->GetNumberOfPoints()-1, new_pt);

    // Extend front point
    pt1 = new_pts->GetPoint(0);
    new_pt[0] = pt1[0];
    new_pt[1] = pt1[1];
    new_pt[2] = pt1[2];
    pt1 = new_pts->GetPoint(1);
    new_pt[0] +=  (new_pt[0] - pt1[0]);
    new_pt[1] +=  (new_pt[1] - pt1[1]);
    new_pt[2] +=  (new_pt[2] - pt1[2]);
    new_pts->SetPoint(0, new_pt);

    new_points->SetPoints(new_pts);
    
    //estimateNewNormals(new_points);
    computeSurfaceLineNormals(new_points);

    return new_points;
  }

  vtkSmartPointer<vtkPolyData> RasterToolPathPlanner::extrudeSplineToSurface(vtkSmartPointer<vtkPolyData> line,
                                                                             double min_extrude_dist )
  {
    static const double EXTRUDE_EXTEND_PERCENTAGE = 1.5;
    static const double RAY_INTERSECTION_TOLERANCE = 0.001;
    vtkSmartPointer<vtkPolyData> new_surface;
    vtkSmartPointer<vtkDataArray> normals = line->GetPointData()->GetNormals();

    if(!normals)
    {
      logError("no normals, cannot create surface from spline");
      return new_surface;
    }

    new_surface = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

    // for each point, insert 2 points, one above and one below, to create a new surface
    double dist_to_surf;
    Eigen::Vector3d closest_point;
    Eigen::Vector3d cell_normal;
    vtkIdType cell_id;
    int sub_index;
    for(int i = 0; i < line->GetNumberOfPoints(); ++i)
    {
      Eigen::Vector3d current_point = Eigen::Vector3d::Zero();
      Eigen::Vector3d current_normal = Eigen::Vector3d::Zero();
      line->GetPoints()->GetPoint(i,current_point.data());
      line->GetPointData()->GetNormals()->GetTuple(i,current_normal.data());
      current_normal.normalize();

      // finding distance and closest point to surface
/*      cell_locator_->FindClosestPoint(current_point.data(),closest_point.data(),cell_id,sub_index,dist_to_surf);
      input_mesh_->GetCellData()->GetNormals()->GetTuple(cell_id,cell_normal.data());
      cell_normal.normalize();*/

      Eigen::Vector3d ray_source_point = min_extrude_dist * current_normal + current_point;
      Eigen::Vector3d ray_target_point = -min_extrude_dist * current_normal + current_point;
      Eigen::Vector3d extruded_point_a, extruded_point_b, intersection_point;
      vtkSmartPointer<vtkPoints> intersection_points = vtkSmartPointer<vtkPoints>::New();
      int res = bsp_tree_->IntersectWithLine(ray_source_point.data(),ray_target_point.data(),
                                             RAY_INTERSECTION_TOLERANCE,
                                             intersection_points,nullptr);

      bool refine_extrude_points = false;
      Eigen::Vector3d dir;
      double dist = 0.0;

      if(res > 0)
      {
        intersection_points->GetPoint(0,intersection_point.data());
        dir = (intersection_point - current_point);
        dist = dir.norm();
        refine_extrude_points = dist > RAY_INTERSECTION_TOLERANCE;
      }

      if(!refine_extrude_points)
      {
        extruded_point_a = ray_source_point;
        extruded_point_b = ray_target_point;
      }
      else
      {
        // slightly past the current point opposite to the ray direction
        extruded_point_a = intersection_point - EXTRUDE_EXTEND_PERCENTAGE * dist * dir.normalized();

        // slightly past the intersection point in the ray direction
        extruded_point_b = current_point + EXTRUDE_EXTEND_PERCENTAGE * dist * dir.normalized();
      }

      if(i < line->GetNumberOfPoints() - 1)
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

      points->InsertNextPoint(extruded_point_a.data());
      points->InsertNextPoint(extruded_point_b.data());
    }

    new_surface->SetPolys(cells);
    new_surface->SetPoints(points);
    return new_surface;
  }

  vtkSmartPointer<vtkPolyData> RasterToolPathPlanner::createSurfaceFromSpline(vtkSmartPointer<vtkPolyData> line,
                                                                              double dist)
  {
    vtkSmartPointer<vtkPolyData> new_surface;
    vtkSmartPointer<vtkDataArray> normals = line->GetPointData()->GetNormals();

    if(!normals)
    {
      logError("no normals, cannot create surface from spline");
      return new_surface;
    }

    new_surface = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

    // for each point, insert 2 points, one above and one below, to create a new surface
    double dist_to_surf;
    for(int i = 0; i < line->GetNumberOfPoints(); ++i)
    {
      Eigen::Vector3d current_point = Eigen::Vector3d::Zero();
      Eigen::Vector3d current_normal = Eigen::Vector3d::Zero();
      line->GetPoints()->GetPoint(i,current_point.data());
      line->GetPointData()->GetNormals()->GetTuple(i,current_normal.data());

      current_normal.normalize();
      Eigen::Vector3d point_below = -dist * current_normal  + current_point;
      Eigen::Vector3d point_above = dist * current_normal + current_point;
      if(i < line->GetNumberOfPoints() - 1)
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

      points->InsertNextPoint( point_above.data());
      points->InsertNextPoint(point_below.data());
    }

    new_surface->SetPolys(cells);
    new_surface->SetPoints(points);
    return new_surface;
  }

  void RasterToolPathPlanner::setCutDirection(double direction [3])
  {
    cut_direction_[0] = direction[0];
    cut_direction_[1] = direction[1];
    cut_direction_[2] = direction[2];
  }
  void RasterToolPathPlanner::setCutCentroid(double centroid [3])
  {
    cut_centroid_[0] = centroid[0];
    cut_centroid_[1] = centroid[1];
    cut_centroid_[2] = centroid[2];
  }

}
