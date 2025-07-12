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

  virtual typename T::ConstPtr create() const = 0;

  virtual void configure(const YAML::Node&) {}
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
