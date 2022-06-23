#include <noether_gui/widgets/tool_path_modifier_widgets.h>
#include "ui_vector3d_editor_widget.h"

#include <noether_tpp/tool_path_modifiers/organization_modifiers.h>
#include <noether_tpp/tool_path_modifiers/waypoint_orientation_modifiers.h>
#include <QFormLayout>
#include <QLabel>
#include <QSpinBox>

namespace noether
{
StandardEdgePathsOrganizationModifierWidget::StandardEdgePathsOrganizationModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent), ui_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);
  ui_->group_box->setTitle("Start Reference");
}

ToolPathModifier::ConstPtr StandardEdgePathsOrganizationModifierWidget::create() const
{
  Eigen::Vector3d start_ref(
      ui_->double_spin_box_x->value(), ui_->double_spin_box_y->value(), ui_->double_spin_box_z->value());
  return std::make_unique<StandardEdgePathsOrganizationModifier>(start_ref);
}

ToolPathModifier::ConstPtr RasterOrganizationModifierWidget::create() const
{
  return std::make_unique<RasterOrganizationModifier>();
}

ToolPathModifier::ConstPtr SnakeOrganizationModifierWidget::create() const
{
  return std::make_unique<SnakeOrganizationModifier>();
}

FixedOrientationModifierWidget::FixedOrientationModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent), ui_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);
  ui_->group_box->setTitle("Reference X Direction");

  // Manually set x value
  ui_->double_spin_box_x->setValue(1.0);
}

ToolPathModifier::ConstPtr FixedOrientationModifierWidget::create() const
{
  Eigen::Vector3d ref_x_dir(
      ui_->double_spin_box_x->value(), ui_->double_spin_box_y->value(), ui_->double_spin_box_z->value());
  ref_x_dir.normalize();
  return std::make_unique<FixedOrientationModifier>(ref_x_dir);
}

ToolPathModifier::ConstPtr DirectionOfTravelOrientationModifierWidget::create() const
{
  return std::make_unique<DirectionOfTravelOrientationModifier>();
}

ToolPathModifier::ConstPtr UniformOrientationModifierWidget::create() const
{
  return std::make_unique<UniformOrientationModifier>();
}

MovingAverageOrientationSmoothingModifierWidget::MovingAverageOrientationSmoothingModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent)
  , layout_(new QFormLayout(this))
  , label_(new QLabel("Window size", this))
  , window_size_(new QSpinBox(this))
{
  layout_->addRow(label_, window_size_);
  window_size_->setMinimum(3);
}

ToolPathModifier::ConstPtr MovingAverageOrientationSmoothingModifierWidget::create() const
{
  return std::make_unique<MovingAverageOrientationSmoothingModifier>(window_size_->value());
}

}  // namespace noether
