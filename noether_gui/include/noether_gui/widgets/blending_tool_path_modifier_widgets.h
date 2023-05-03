#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_modifier.h>

class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;

namespace noether
{
class AngledOrientationToolPathModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  AngledOrientationToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

private:
  QDoubleSpinBox* angle_offset_;
  QDoubleSpinBox* tool_radius_;
  QCheckBox* flip_sign_;
};

class LeadInToolPathModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  LeadInToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

private:
  QDoubleSpinBox* lead_in_angle_;
  QDoubleSpinBox* lead_in_arc_radius_;
  QSpinBox* lead_in_num_of_points_;
};

class LeadOutToolPathModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  LeadOutToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

private:
  QDoubleSpinBox* lead_out_angle_;
  QDoubleSpinBox* lead_out_arc_radius_;
  QSpinBox* lead_out_num_of_points_;
};

}  // namespace noether
