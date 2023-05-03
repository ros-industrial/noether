#include <noether_gui/plugin_interface.h>
#include <noether_gui/widgets/direction_generator_widgets.h>
#include <noether_gui/widgets/origin_generator_widgets.h>
#include <noether_gui/widgets/tool_path_modifier_widgets.h>
#include <noether_gui/widgets/blending_tool_path_modifier_widgets.h>
#include <noether_gui/widgets/raster_planner_widget.h>
#include <noether_gui/widgets/plane_slicer_raster_planner_widget.h>

#include <QWidget>

namespace noether
{
template <typename T, typename BaseT>
struct WidgetPluginImpl : WidgetPlugin<BaseT>
{
  QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override final
  {
    return new T(parent);
  }
};

// Direction Generators
struct FixedDirectionGeneratorWidgetPlugin : DirectionGeneratorWidgetPlugin
{
  QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override final
  {
    try
    {
      auto x = config["x"].as<double>();
      auto y = config["y"].as<double>();
      auto z = config["z"].as<double>();
      Eigen::Vector3d dir(x, y, z);
      dir.normalize();
      return new FixedDirectionGeneratorWidget(parent, dir);
    }
    catch (const YAML::Exception&)
    {
      return new FixedDirectionGeneratorWidget(parent);
    }
  }
};

struct PrincipalAxisDirectionGeneratorWidgetPlugin : DirectionGeneratorWidgetPlugin
{
  QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override final
  {
    try
    {
      return new PrincipalAxisDirectionGeneratorWidget(parent, config["rotation_offset"].as<double>());
    }
    catch (const YAML::Exception&)
    {
      return new PrincipalAxisDirectionGeneratorWidget(parent);
    }
  }
};

// Origin Generators
struct FixedOriginGeneratorWidgetPlugin : OriginGeneratorWidgetPlugin
{
  QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override final
  {
    try
    {
      auto x = config["x"].as<double>();
      auto y = config["y"].as<double>();
      auto z = config["z"].as<double>();
      Eigen::Vector3d dir(x, y, z);
      dir.normalize();
      return new FixedOriginGeneratorWidget(parent, dir);
    }
    catch (const YAML::Exception&)
    {
      return new FixedOriginGeneratorWidget(parent);
    }
  }
};

using CentroidOriginGeneratorWidgetPlugin = WidgetPluginImpl<CentroidOriginGeneratorWidget, OriginGeneratorWidget>;

using AABBOriginGeneratorWidgetPlugin = WidgetPluginImpl<AABBOriginGeneratorWidget, OriginGeneratorWidget>;

// Tool Path Modifiers
using StandardEdgePathsOrganizationModifierWidgetPlugin =
    WidgetPluginImpl<StandardEdgePathsOrganizationModifierWidget, ToolPathModifierWidget>;

using RasterOrganizationModifierWidgetPlugin =
    WidgetPluginImpl<RasterOrganizationModifierWidget, ToolPathModifierWidget>;

using SnakeOrganizationModifierWidgetPlugin = WidgetPluginImpl<SnakeOrganizationModifierWidget, ToolPathModifierWidget>;

using FixedOrientationModifierWidgetPlugin = WidgetPluginImpl<FixedOrientationModifierWidget, ToolPathModifierWidget>;

using DirectionOfTravelOrientationModifierWidgetPlugin =
    WidgetPluginImpl<DirectionOfTravelOrientationModifierWidget, ToolPathModifierWidget>;

using UniformOrientationModifierWidgetPlugin =
    WidgetPluginImpl<UniformOrientationModifierWidget, ToolPathModifierWidget>;

using MovingAverageOrientationSmoothingModifierWidgetPlugin =
    WidgetPluginImpl<MovingAverageOrientationSmoothingModifierWidget, ToolPathModifierWidget>;

using ToolDragOrientationToolPathModifierWidgetPlugin =
    WidgetPluginImpl<ToolDragOrientationToolPathModifierWidget, ToolPathModifierWidget>;

using LeadInToolPathModifierWidgetPlugin =
    WidgetPluginImpl<CircularLeadInToolPathModifierWidget, ToolPathModifierWidget>;

using LeadOutToolPathModifierWidgetPlugin =
    WidgetPluginImpl<CircularLeadOutToolPathModifierWidget, ToolPathModifierWidget>;

// Raster Tool Path Planners
struct PlaneSlicerRasterPlannerWidgetPlugin : ToolPathPlannerWidgetPlugin
{
  QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override final
  {
    boost_plugin_loader::PluginLoader loader;
    loader.search_libraries.insert(NOETHER_GUI_PLUGINS);

    try
    {
      auto line_spacing = config["line_spacing"].as<double>();
      auto point_spacing = config["point_spacing"].as<double>();
      auto min_hole_size = config["min_hole_size"].as<double>();
      auto search_radius = config["search_radius"].as<double>();
      auto min_segment_size = config["min_segment_size"].as<double>();
      return new PlaneSlicerRasterPlannerWidget(
          std::move(loader), parent, line_spacing, point_spacing, min_hole_size, search_radius, min_segment_size);
    }
    catch (const YAML::Exception&)
    {
      return new PlaneSlicerRasterPlannerWidget(std::move(loader), parent);
    }
  }
};

}  // namespace noether

EXPORT_DIRECTION_GENERATOR_WIDGET_PLUGIN(noether::FixedDirectionGeneratorWidgetPlugin, FixedDirectionGenerator)
EXPORT_DIRECTION_GENERATOR_WIDGET_PLUGIN(noether::PrincipalAxisDirectionGeneratorWidgetPlugin,
                                         PrincipalAxisDirectionGenerator)

EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::FixedOriginGeneratorWidgetPlugin, FixedOriginGenerator)
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::CentroidOriginGeneratorWidgetPlugin, CentroidOriginGenerator)
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::AABBOriginGeneratorWidgetPlugin, AABBOriginGenerator)

EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::StandardEdgePathsOrganizationModifierWidgetPlugin,
                                        StandardEdgePathsOrganizationModifier)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::RasterOrganizationModifierWidgetPlugin, RasterOrganizationModifier)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::SnakeOrganizationModifierWidgetPlugin, SnakeOrganizationModifier)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::FixedOrientationModifierWidgetPlugin, FixedOrientationModifier)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::DirectionOfTravelOrientationModifierWidgetPlugin,
                                        DirectionOfTravelOrientationModifier)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::UniformOrientationModifierWidgetPlugin, UniformOrientationModifier)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::MovingAverageOrientationSmoothingModifierWidgetPlugin,
                                        MovingAverageOrientationSmoothingModifier)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::ToolDragOrientationToolPathModifierWidgetPlugin,
                                        ToolDragOrientationToolPathModifier)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::LeadInToolPathModifierWidgetPlugin, LeadInModifier)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::LeadOutToolPathModifierWidgetPlugin, LeadOutModifier)

EXPORT_TPP_WIDGET_PLUGIN(noether::PlaneSlicerRasterPlannerWidgetPlugin, PlaneSlicerRasterPlanner)
