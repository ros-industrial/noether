#pragma once

#include <QWidget>
#include <yaml-cpp/yaml.h>

namespace noether
{
class ToolPathPlanner;
class ToolPathModifier;
class DirectionGenerator;
class OriginGenerator;
class MeshModifier;

/**
 * @ingroup gui_interfaces_widgets
 * @brief Base class for a widget "factory" that can produce classes of a specifiable type
 */
template <typename T>
class BaseWidget : public QWidget
{
public:
  BaseWidget(QWidget* parent = nullptr) : QWidget(parent) {}

  /**
   * @brief Creates a tool path planning component of type `T` (e.g., mesh modifier, tool path planner, tool path
   * modifier)
   */
  virtual typename T::ConstPtr create() const = 0;

  /**
   * @brief Configures the elements of the widget from a YAML node
   */
  virtual void configure(const YAML::Node&) {}

  /**
   * @brief Saves the configuration of the widget to a YAML node
   */
  virtual void save(YAML::Node&) const {}
};

/**
 * @ingroup gui_interfaces_widgets
 * @brief Widget for configuring instances of ToolPathPlanner
 */
using ToolPathPlannerWidget = BaseWidget<ToolPathPlanner>;

/**
 * @ingroup gui_interfaces_widgets
 * @brief Widget for configuring instances of DirectionGenerator
 */
using DirectionGeneratorWidget = BaseWidget<DirectionGenerator>;

/**
 * @ingroup gui_interfaces_widgets
 * @brief Widget for configuring instances of OriginGenerator
 */
using OriginGeneratorWidget = BaseWidget<OriginGenerator>;

/**
 * @ingroup gui_interfaces_widgets
 * @brief Widget for configuring instances of ToolPathModifier
 */
using ToolPathModifierWidget = BaseWidget<ToolPathModifier>;

/**
 * @ingroup gui_interfaces_widgets
 * @brief Widget for configuring instances of MeshModifier
 */
using MeshModifierWidget = BaseWidget<MeshModifier>;

}  // namespace noether
