#include <noether_tpp/tool_path_modifiers/organization_modifiers.h>

#include <numeric>

namespace noether
{
/** @brief Estimates the direction of travel of a tool path by averaging the vectors between adjacent waypoints */
static Eigen::Vector3d estimateToolPathDirection(const ToolPath& tool_path)
{
  Eigen::Vector3d avg_dir = Eigen::Vector3d::Zero();
  int n = 0;
  for (const ToolPathSegment& seg : tool_path)
  {
    if (seg.size() > 1)
    {
      for (std::size_t i = 0; i < seg.size() - 1; ++i)
      {
        avg_dir += (seg.at(i + 1).translation() - seg.at(i).translation()).normalized();
        ++n;
      }
    }
  }

  if (n < 1)
    throw std::runtime_error("Insufficient number of points to calculate average tool path direction");

  avg_dir /= static_cast<double>(n);

  return avg_dir.normalized();
}

/**
 * @brief Estimates the raster direction of a set of tool paths by returning the component of the vector from the first
 * point in the first segment of the first tool path) to the first waypoint in the first segment of the last tool path
 * that is perpendicular to the nominal tool path direction
 */
static Eigen::Vector3d estimateRasterDirection(const ToolPaths& tool_paths,
                                               const Eigen::Vector3d& reference_tool_path_dir)
{
  // First waypoint in the first segment of the first tool path
  const Eigen::Isometry3d first_wp = tool_paths.at(0).at(0).at(0);
  // First waypoint in the first segment of the last tool path
  const Eigen::Isometry3d first_wp_last_path = tool_paths.at(tool_paths.size() - 1).at(0).at(0);

  // Normalize the reference tool path direction
  const Eigen::Vector3d norm_ref_tool_path_dir = reference_tool_path_dir.normalized();

  // Get the vector between the two points
  Eigen::Vector3d raster_dir = first_wp_last_path.translation() - first_wp.translation();

  // Subtract the component of this vector that is parallel to the reference tool path direction
  raster_dir -= raster_dir.dot(norm_ref_tool_path_dir) * norm_ref_tool_path_dir;
  return raster_dir.normalized();
}

ToolPaths RasterOrganizationModifier::modify(ToolPaths tool_paths) const
{
  ToolPathSegment first_seg = tool_paths.at(0).at(0);
  Eigen::Isometry3d first_wp = first_seg.at(0);
  Eigen::Isometry3d last_wp_first_seg = first_seg.at(first_seg.size() - 1);
  const Eigen::Vector3d reference_segment_dir = estimateToolPathDirection(tool_paths.at(0));

  for (ToolPath& tool_path : tool_paths)
  {
    // Sort the waypoints within each tool path segment by their distance along the reference direction
    for (ToolPathSegment& segment : tool_path)
    {
      std::sort(segment.begin(),
                segment.end(),
                [segment, reference_segment_dir](const ToolPathWaypoint& a, const ToolPathWaypoint& b) {
                  Eigen::Vector3d diff_from_start_b = b.translation() - segment.at(0).translation();
                  Eigen::Vector3d diff_from_start_a = a.translation() - segment.at(0).translation();
                  return diff_from_start_a.dot(reference_segment_dir) < diff_from_start_b.dot(reference_segment_dir);
                });
    }

    // Sort the tool path segments within each tool path by the distance of their first waypoints along the reference
    // direction
    std::sort(tool_path.begin(),
              tool_path.end(),
              [tool_path, reference_segment_dir](const ToolPathSegment& a, const ToolPathSegment& b) {
                Eigen::Vector3d diff_from_start_b = b.at(0).translation() - tool_path.at(0).at(0).translation();
                Eigen::Vector3d diff_from_start_a = a.at(0).translation() - tool_path.at(0).at(0).translation();
                return diff_from_start_a.dot(reference_segment_dir) < diff_from_start_b.dot(reference_segment_dir);
              });
  }

  // Sort the tool paths by their distance along a vector that is perpendicular to the reference direction of travel
  const Eigen::Vector3d reference_tool_paths_dir = estimateRasterDirection(tool_paths, reference_segment_dir);
  std::sort(tool_paths.begin(),
            tool_paths.end(),
            [tool_paths, reference_tool_paths_dir, first_wp](const ToolPath& a, const ToolPath& b) {
              Eigen::Vector3d diff_from_start_b = b.at(0).at(0).translation() - first_wp.translation();
              Eigen::Vector3d diff_from_start_a = a.at(0).at(0).translation() - first_wp.translation();
              return diff_from_start_a.dot(reference_tool_paths_dir) < diff_from_start_b.dot(reference_tool_paths_dir);
            });

  return tool_paths;
}

ToolPaths SnakeOrganizationModifier::modify(ToolPaths tool_paths) const
{
  // Create a raster
  RasterOrganizationModifier raster;
  tool_paths = raster.modify(tool_paths);

  // Re-sort the tool paths at odd indices
  for (std::size_t i = 1; i < tool_paths.size(); i += 2)
  {
    ToolPath& tool_path = tool_paths.at(i);

    // Sort waypoints in each tool path segment by descending x-axis value (i.e. right to left)
    for (ToolPathSegment& segment : tool_path)
    {
      std::reverse(segment.begin(), segment.end());
    }

    std::reverse(tool_path.begin(), tool_path.end());
  }

  return tool_paths;
}

/**
 * @brief Organizes the tool path segments within each tool path in the container such that the beginning of a tool path
 * is as close as possible to the end of the previous tool path (or a reference position, in the case of the first tool
 * path)
 * @details The primary intent of this function is to organize tool path segments for edge tool paths; these segments
 * represent segments of an edge and should be arranged in a sequential and connected order.
 */
static std::vector<std::size_t> sortToolPathBySegmentConnection(const ToolPath& path,
                                                                const Eigen::Vector3d& start_reference)
{
  std::vector<std::size_t> segment_order;
  segment_order.reserve(path.size());

  while (segment_order.size() < path.size())
  {
    // Get the indices of the yet unordered segments
    std::vector<std::size_t> remaining_segment_indices;
    {
      std::vector<std::size_t> order(path.size());
      // Fill the order vector with increasing integers, starting at 0
      std::iota(order.begin(), order.end(), 0);
      // Extract only the indices that have not been used so far into remaining_segment_indices
      std::set_difference(order.begin(),
                          order.end(),
                          segment_order.begin(),
                          segment_order.end(),
                          std::back_inserter(remaining_segment_indices));
    }

    if (remaining_segment_indices.size() == 1)
    {
      segment_order.push_back(remaining_segment_indices.front());
    }
    else
    {
      // Get the reference point with which to compare the segment start positions
      Eigen::Vector3d ref;
      if (segment_order.empty())
      {
        // Use the reference start position if no segments have been ordered yet
        ref = start_reference;
      }
      else
      {
        // Otherwise use the position of the last waypoint in the last ordered segment
        ref = path.at(segment_order.back()).back().translation();
      }

      // Find the segment whose first waypoint is closest to the reference position
      double min_dist = std::numeric_limits<double>::max();
      std::size_t min_index = 0;
      for (std::size_t idx : remaining_segment_indices)
      {
        double dist = (path.at(idx).at(0).translation() - ref).norm();
        if (dist < min_dist)
        {
          min_dist = dist;
          min_index = idx;
        }
      }

      // Add the index of that segment to the output variable
      segment_order.push_back(min_index);
    }
  }

  return segment_order;
}

/**
 * @brief Helper function for creating a re-ordered copy of a vector
 */
template <typename T>
static std::vector<T> reorder(const std::vector<T>& input, const std::vector<std::size_t>& order)
{
  std::vector<T> output;
  output.reserve(input.size());

  for (std::size_t idx : order)
  {
    output.push_back(input.at(idx));
  }

  return output;
}

/**
 * @brief Organizes the container of tool paths according to the length of all the tool path segments in the tool path
 * @details The tool paths are sorted from longest to shortest. The organization of the tool path segments within each
 * tool path and the waypoints within each tool path segment are not changed. The primary intent of this modifier is for
 * sorting edge tool paths by length.
 */
static ToolPaths sortToolPathsByLength(const ToolPaths& tool_paths)
{
  std::vector<double> path_lengths;
  path_lengths.reserve(tool_paths.size());

  for (const ToolPath& path : tool_paths)
  {
    double path_length = 0.0;

    for (const ToolPathSegment& segment : path)
    {
      for (std::size_t i = 0; i < segment.size() - 1; ++i)
      {
        const ToolPathWaypoint& first = segment.at(i);
        const ToolPathWaypoint& second = segment.at(i + 1);
        path_length += (first.inverse() * second).translation().norm();
      }
    }

    path_lengths.push_back(path_length);
  }

  // Argsort the path lengths from largest to shortest
  std::vector<std::size_t> order(path_lengths.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&path_lengths](const std::size_t& a, const std::size_t& b) {
    return path_lengths.at(a) > path_lengths.at(b);
  });

  // Copy the tool paths in the new order
  return reorder(tool_paths, order);
}

StandardEdgePathsOrganizationModifier::StandardEdgePathsOrganizationModifier(const Eigen::Vector3d& start_reference)
  : start_reference_(start_reference)
{
}

ToolPaths StandardEdgePathsOrganizationModifier::modify(ToolPaths tool_paths) const
{
  // Sort the tool path segments such that the first segment is the one whose first waypoint is nearest the start
  // reference point and subsequent segments start near the end of the previous segment
  ToolPaths ordered_tool_paths;
  ordered_tool_paths.reserve(tool_paths.size());

  for (ToolPath& path : tool_paths)
  {
    // Get the desired order of the tool path segments
    std::vector<std::size_t> segment_order = sortToolPathBySegmentConnection(path, start_reference_);

    // Create a new tool path with the segments in the desired order
    ordered_tool_paths.push_back(reorder(path, segment_order));
  }

  // Sort the tool paths by length
  return sortToolPathsByLength(ordered_tool_paths);
}

}  // namespace noether
