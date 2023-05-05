#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_modifier.h>

class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;

namespace noether
{
class ToolDragOrientationToolPathModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  ToolDragOrientationToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

private:
  QDoubleSpinBox* angle_offset_;
  QDoubleSpinBox* tool_radius_;
};

class CircularLeadInToolPathModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  CircularLeadInToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

private:
  QDoubleSpinBox* arc_angle_;
  QDoubleSpinBox* arc_radius_;
  QSpinBox* n_points_;
};

class CircularLeadOutToolPathModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  CircularLeadOutToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

private:
  QDoubleSpinBox* arc_angle_;
  QDoubleSpinBox* arc_radius_;
  QSpinBox* n_points;
};

}  // namespace noether
