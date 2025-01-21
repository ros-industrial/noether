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
 * @ingroup gui_interfaces
 * @brief Base class for a widget "factory" that can produce classes of a specifiable type
 */
template <typename T>
class BaseWidget : public QWidget
{
public:
  BaseWidget(QWidget* parent = nullptr) : QWidget(parent) {}

  virtual typename T::ConstPtr create() const = 0;

  virtual void configure(const YAML::Node&) {}
  virtual void save(YAML::Node&) const {}
};

/**
 * @ingroup gui_interfaces_widgets
 * @brief Widget for configuring ToolPathPlanner implementations
 */
using ToolPathPlannerWidget = BaseWidget<ToolPathPlanner>;

/**
 * @ingroup gui_interfaces_widgets
 * @brief Widget for configuring DirectionGenerator implementations
 */
using DirectionGeneratorWidget = BaseWidget<DirectionGenerator>;

/**
 * @ingroup gui_interfaces_widgets
 * @brief Widget for configuring OriginGenerator implementations
 */
using OriginGeneratorWidget = BaseWidget<OriginGenerator>;

/**
 * @ingroup gui_interfaces_widgets
 * @brief Widget for configuring ToolPathModifier implementations
 */
using ToolPathModifierWidget = BaseWidget<ToolPathModifier>;

/**
 * @ingroup gui_interfaces_widgets
 * @brief Widget for configuring MeshModifier implementations
 */
using MeshModifierWidget = BaseWidget<MeshModifier>;

}  // namespace noether
