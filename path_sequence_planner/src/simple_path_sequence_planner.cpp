/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <algorithm>
#include <path_sequence_planner/simple_path_sequence_planner.h>
#include <tool_path_planner/utilities.h>
#include <vtk_viewer/vtk_utils.h>

namespace path_sequence_planner
{

void SimplePathSequencePlanner::linkPaths()
{
  bool insert_front = false;
  int last_index = 0;

  while(indices_.size() != paths_.size())
  {
    if(indices_.size() == 0)
    {
      indices_.push_back(1);
    }
    else
    {
      // find next nearest point
      if(insert_front)
      {
        last_index = indices_.front();
      }
      else
      {
        last_index = indices_.back();
      }

      int next_index = findNextNearestPath(paths_, indices_, last_index, insert_front);



      if(indices_.size() > 1)
      {
        // check the distance of next_index with front and back to make sure it is in the right location
        double* front_pt = paths_[indices_.front()].line->GetPoints()->GetPoint(0);
        double* end_pt = paths_[indices_.back()].line->GetPoints()->GetPoint(paths_[indices_.back()].line->GetPoints()->GetNumberOfPoints()-1);
        double* next_pt = paths_[next_index].line->GetPoints()->GetPoint(paths_[next_index].line->GetPoints()->GetNumberOfPoints()-1);

        double front_dist1 = vtk_viewer::pt_dist(front_pt, next_pt);
        double back_dist1 = vtk_viewer::pt_dist(end_pt, next_pt);

        next_pt = paths_[next_index].line->GetPoints()->GetPoint(0);

        double front_dist2 = vtk_viewer::pt_dist(front_pt, next_pt);
        double back_dist2 = vtk_viewer::pt_dist(end_pt, next_pt);

        // If the next path found is closer to the opposite side, flip which end we are adding paths to
        bool flip = (front_dist1 < front_dist2 ? front_dist1 : front_dist2) < (back_dist1 < back_dist2 ? back_dist1 : back_dist2) ? true : false;

        // flip insert side and start from the beginning (finding the next closest path)
        if(flip != insert_front)
        {
          insert_front = flip;
          continue;
        }
      }

      // insert the index found of the next closest line
      if(next_index >= 0)
      {
        if(insert_front)
        {
          indices_.insert(indices_.begin(), next_index);
        }
        else
        {
          indices_.push_back(next_index);
        }

        double* last_pt;
        if(insert_front)
        {
          last_pt = paths_[last_index].line->GetPoints()->GetPoint( 0 );
        }
        else
        {
          last_pt = paths_[last_index].line->GetPoints()->GetPoint( paths_[last_index].line->GetPoints()->GetNumberOfPoints() - 1 );
        }

        // get first last point of of the line and determine if it needs to be flipped
        double* pt1 = paths_[next_index].line->GetPoints()->GetPoint(0);
        double dist1 = vtk_viewer::pt_dist(pt1, last_pt);

        // find distance between last point and the end points of the next line
        int indx = (paths_[next_index].line->GetPoints()->GetNumberOfPoints()) - 1;
        double* pt2 = paths_[next_index].line->GetPoints()->GetPoint( indx );
        double dist2 = vtk_viewer::pt_dist(pt2, last_pt);

        // If the distance is shorter, flip the order of the next path
        if(dist2 < dist1 && !insert_front)
        {
          tool_path_planner::flipPointOrder(paths_[next_index]);
        }
        else if (dist1 < dist2 && insert_front)
        {
          tool_path_planner::flipPointOrder(paths_[next_index]);
        }
      }

    }
  }
}

int SimplePathSequencePlanner::findNextNearestPath(std::vector<tool_path_planner::ProcessPath> paths,
                                             std::vector<int> used_indices, int last_path, bool front)
{
  double* last_pt;
  //find next nearest point
  if(front)
  {
    last_pt = paths[last_path].line->GetPoints()->GetPoint( 0 );
  }
  else
  {
    last_pt = paths[last_path].line->GetPoints()->GetPoint( paths[last_path].line->GetPoints()->GetNumberOfPoints() - 1 );
  }

  int min_index = -1;
  double min_dist = std::numeric_limits<double>::max();

  for (int j = 0; j < paths.size(); ++j)
  {
    // If the current index is aleady used, skip it
    if( std::find(used_indices.begin(), used_indices.end(), j) != used_indices.end() )
    {
      continue;
    }

    // get first and last point of line j
    double* pt1 = paths[j].line->GetPoints()->GetPoint(0);
    double dist1 = vtk_viewer::pt_dist(pt1, last_pt);

    // find distance between last point and the end points of the next line
    double* pt2 = paths[j].line->GetPoints()->GetPoint( paths[j].line->GetPoints()->GetNumberOfPoints() - 1 );
    double dist2 = vtk_viewer::pt_dist(pt2, last_pt);

    if(dist1 < min_dist || dist2 < min_dist)
    {
      min_index = j;
      min_dist = (dist1 < dist2 ? dist1 : dist2);
    }
  }

  return min_index;
}

}

