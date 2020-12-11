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
void SimplePathSequencePlanner::setPaths(tool_path_planner::ToolPaths paths)
{
  paths_ = paths;
  indices_.clear();
}

tool_path_planner::ToolPaths SimplePathSequencePlanner::getPaths() const { return paths_; }

std::vector<std::size_t> SimplePathSequencePlanner::getIndices() const { return indices_; }

void SimplePathSequencePlanner::linkPaths()
{
  bool insert_front = false;
  std::size_t last_index = 0;

  while (indices_.size() != paths_.size())
  {
    if (indices_.size() == 0)
    {
      indices_.push_back(1);
    }
    else
    {
      // find next nearest point
      if (insert_front)
      {
        last_index = indices_.front();
      }
      else
      {
        last_index = indices_.back();
      }

      long next_index = findNextNearestPath(paths_, indices_, last_index, insert_front);

      if (next_index < 0)
        return;

      if (indices_.size() > 1)
      {
        // check the distance of next_index with front and back to make sure it is in the right location
        const Eigen::Isometry3d& front_pt = paths_[indices_.front()].front().front();
        const Eigen::Isometry3d& end_pt = paths_[indices_.back()].back().back();
        Eigen::Isometry3d next_pt = paths_[static_cast<std::size_t>(next_index)].back().back();

        double front_dist1 = (next_pt.translation() - front_pt.translation()).norm();
        double back_dist1 = (next_pt.translation() - end_pt.translation()).norm();

        next_pt = paths_[static_cast<std::size_t>(next_index)].front().front();

        double front_dist2 = (next_pt.translation() - front_pt.translation()).norm();
        double back_dist2 = (next_pt.translation() - end_pt.translation()).norm();

        // If the next path found is closer to the opposite side, flip which end we are adding paths to
        bool flip = (front_dist1 < front_dist2 ? front_dist1 : front_dist2) <
                            (back_dist1 < back_dist2 ? back_dist1 : back_dist2) ?
                        true :
                        false;

        // flip insert side and start from the beginning (finding the next closest path)
        if (flip != insert_front)
        {
          insert_front = flip;
          continue;
        }
      }

      // insert the index found of the next closest line
      if (next_index >= 0)
      {
        if (insert_front)
          indices_.insert(indices_.begin(), static_cast<std::size_t>(next_index));
        else
          indices_.push_back(static_cast<std::size_t>(next_index));

        Eigen::Isometry3d last_pt;
        if (insert_front)
          last_pt = paths_[last_index].front().front();
        else
          last_pt = paths_[last_index].back().back();

        // get first last point of of the line and determine if it needs to be flipped
        const Eigen::Isometry3d& pt1 = paths_[static_cast<std::size_t>(next_index)].front().front();
        double dist1 = (pt1.translation() - last_pt.translation()).norm();

        // find distance between last point and the end points of the next line
        const Eigen::Isometry3d& pt2 = paths_[static_cast<std::size_t>(next_index)].back().back();
        double dist2 = (pt2.translation() - last_pt.translation()).norm();

        // If the distance is shorter, flip the order of the next path
        if (dist2 < dist1 && !insert_front)
        {
          tool_path_planner::flipPointOrder(paths_[static_cast<std::size_t>(next_index)]);
        }
        else if (dist1 < dist2 && insert_front)
        {
          tool_path_planner::flipPointOrder(paths_[static_cast<std::size_t>(next_index)]);
        }
      }
    }
  }
}

long SimplePathSequencePlanner::findNextNearestPath(tool_path_planner::ToolPaths paths,
                                                    std::vector<std::size_t> used_indices,
                                                    std::size_t last_path,
                                                    bool front)
{
  Eigen::Isometry3d last_pt;
  // find next nearest point
  if (front)
    last_pt = paths[last_path].front().front();
  else
    last_pt = paths[last_path].back().back();

  long min_index = -1;
  double min_dist = std::numeric_limits<double>::max();

  for (std::size_t j = 0; j < paths.size(); ++j)
  {
    // If the current index is aleady used, skip it
    if (std::find(used_indices.begin(), used_indices.end(), j) != used_indices.end())
      continue;

    // get first and last point of line j
    Eigen::Isometry3d pt1 = paths[j].front().front();
    double dist1 = (pt1.translation() - last_pt.translation()).norm();

    // find distance between last point and the end points of the next line
    Eigen::Isometry3d pt2 = paths[j].back().back();
    double dist2 = (pt2.translation() - last_pt.translation()).norm();

    if (dist1 < min_dist || dist2 < min_dist)
    {
      min_index = static_cast<long>(j);
      min_dist = (dist1 < dist2 ? dist1 : dist2);
    }
  }

  return min_index;
}

}  // namespace path_sequence_planner
