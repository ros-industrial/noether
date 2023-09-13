#include <noether_gui/widgets/tool_path_modifiers/linear_departure_modifier_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_modifiers/linear_departure_modifier.h>
#include "ui_vector3d_editor_widget.h"
#include "ui_linear_approach_modifier_widget.h"

namespace noether
{
ToolPathModifier::ConstPtr LinearDepartureToolPathModifierWidget::create() const
{
  if (ui_->combo_box_menu->currentIndex() == 0)
  {
    auto axis = static_cast<LinearDepartureModifier::Axis>(ui_->combo_box_axis->currentIndex());
    return std::make_unique<LinearDepartureModifier>(
        ui_->double_spin_box_offset->value(), axis, ui_->spin_box_points->value());
  }
  else
  {
    Eigen::Vector3d dir(vector_editor_ui_->double_spin_box_x->value(),
                        vector_editor_ui_->double_spin_box_y->value(),
                        vector_editor_ui_->double_spin_box_z->value());
    return std::make_unique<LinearDepartureModifier>(dir, ui_->spin_box_points->value());
  }
}

}  // namespace noether
