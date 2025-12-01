#pragma once

#include <noether_tpp/core/tool_path_planner.h>

namespace noether
{
/**
 * @ingroup tool_path_planners
 * @brief Runs multiple tool path planners serially and concatenates their outputs together
 */
class MultiToolPathPlanner : public ToolPathPlanner
{
public:
  MultiToolPathPlanner(MultiToolPathPlanner&&) = delete;
  MultiToolPathPlanner(const MultiToolPathPlanner&) = delete;

  MultiToolPathPlanner& operator=(const MultiToolPathPlanner&) = delete;
  MultiToolPathPlanner& operator=(MultiToolPathPlanner&&) = delete;

  explicit MultiToolPathPlanner(std::vector<ToolPathPlanner::ConstPtr>&& planners);

  ToolPaths plan(const pcl::PolygonMesh& mesh) const override;

protected:
  const std::vector<ToolPathPlanner::ConstPtr> planners_;
};

}  // namespace noether
