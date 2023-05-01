#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_modifier.h>

class QDoubleSpinBox;
class QCheckBox;

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

}  // namespace noether
