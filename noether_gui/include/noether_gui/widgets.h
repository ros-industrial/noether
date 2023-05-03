#pragma once

#include <QWidget>

namespace YAML
{
class Node;
}

namespace noether
{
class ToolPathPlanner;
class ToolPathModifier;
class DirectionGenerator;
class OriginGenerator;
class MeshModifier;

/**
 * @brief Base class for a widget "factory" that can produce classes of a specifiable type
 */
template <typename T>
class BaseWidget : public QWidget
{
public:
  BaseWidget(QWidget* parent = nullptr) : QWidget(parent) {}

  virtual typename T::ConstPtr create() const = 0;

  virtual void fromYAML(const YAML::Node&) {}
};

using ToolPathPlannerWidget = BaseWidget<ToolPathPlanner>;
using DirectionGeneratorWidget = BaseWidget<DirectionGenerator>;
using OriginGeneratorWidget = BaseWidget<OriginGenerator>;
using ToolPathModifierWidget = BaseWidget<ToolPathModifier>;
using MeshModifierWidget = BaseWidget<MeshModifier>;

}  // namespace noether
