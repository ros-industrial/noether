/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <algorithm>
#include <path_sequence_planner/path_sequence_planner.h>

namespace path_sequence_planner
{

void PathSequencePlanner::linkPaths()
{
  int last_index = 0;
  for(int i = 0; i < paths_.size(); ++i)
  {
    if(i == 0)
    {
      indices_.push_back(i);
    }
    else
    {
      //find next nearest point
      last_index = indices_.back();
      double* last_pt = paths_[last_index].line->GetPoints()->GetPoint( paths_[last_index].line->GetPoints()->GetNumberOfPoints() - 1 );
      int min_index = -1;
      double min_dist = std::numeric_limits<double>::max();

      for (int j = 0; j < paths_.size(); ++j)
      {
        // If the current index is aleady used, skip it
        if( std::find(indices_.begin(), indices_.end(), j) != indices_.end() )
        {
          continue;
        }

        // get first and last point of line j
        double* pt1 = paths_[j].line->GetPoints()->GetPoint(0);
        double dist1 = tool_path_planner::pt_dist(pt1, last_pt);

        // find distance between last point and the end points of the next line
        double* pt2 = paths_[j].line->GetPoints()->GetPoint( paths_[j].line->GetPoints()->GetNumberOfPoints() - 1 );
        double dist2 = tool_path_planner::pt_dist(pt2, last_pt);

        if(dist1 < min_dist || dist2 < min_dist)
        {
          min_index = j;
          min_dist = (dist1 < dist2 ? dist1 : dist2);
        }
      }

      // insert the index found of the next closest line
      if(min_index >= 0)
      {
        indices_.push_back(min_index);

        // get first last point of of the line and determine if it needs to be flipped
        double* pt1 = paths_[min_index].line->GetPoints()->GetPoint(0);
        double dist1 = tool_path_planner::pt_dist(pt1, last_pt);

        // find distance between last point and the end points of the next line
        int indx = (paths_[min_index].line->GetPoints()->GetNumberOfPoints()) - 1;
        double* pt2 = paths_[min_index].line->GetPoints()->GetPoint( indx );
        double dist2 = tool_path_planner::pt_dist(pt2, last_pt);

        cout << "dist1/dist2: " << dist1 << " " << dist2 << "\n";
        if(dist2 < dist1)
        {
          tool_path_planner::flipPointOrder(paths_[min_index]);
        }
      }

    }
  }
}

}
