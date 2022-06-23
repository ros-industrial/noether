#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/core/tool_path_modifier.h>

class QFormLayout;
class QLabel;
class QSpinBox;

namespace Ui
{
class Vector3dEditor;
}

namespace noether
{
class StandardEdgePathsOrganizationModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  StandardEdgePathsOrganizationModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

private:
  Ui::Vector3dEditor* ui_;
};

struct RasterOrganizationModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  using ToolPathModifierWidget::ToolPathModifierWidget;

  ToolPathModifier::ConstPtr create() const override;
};

struct SnakeOrganizationModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  using ToolPathModifierWidget::ToolPathModifierWidget;

  ToolPathModifier::ConstPtr create() const override;
};

class FixedOrientationModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  FixedOrientationModifierWidget(QWidget* parent = nullptr);
  ToolPathModifier::ConstPtr create() const override;

private:
  Ui::Vector3dEditor* ui_;
};

struct DirectionOfTravelOrientationModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
  using ToolPathModifierWidget::ToolPathModifierWidget;
  ToolPathModifier::ConstPtr create() const override;
};

struct UniformOrientationModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
  using ToolPathModifierWidget::ToolPathModifierWidget;
  ToolPathModifier::ConstPtr create() const override;
};

class MovingAverageOrientationSmoothingModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  MovingAverageOrientationSmoothingModifierWidget(QWidget* parent = nullptr);
  ToolPathModifier::ConstPtr create() const override;

private:
  QFormLayout* layout_;
  QLabel* label_;
  QSpinBox* window_size_;
};

}  // namespace noether
