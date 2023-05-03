#include <noether_gui/widgets/blending_tool_path_modifier_widgets.h>
#include <noether_tpp/tool_path_modifiers/blending_modifiers.h>

#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>

namespace noether
{
AngledOrientationToolPathModifierWidget::AngledOrientationToolPathModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent)
{
  auto layout = new QFormLayout(this);

  // Angle between the grinder and the media being ground
  angle_offset_ = new QDoubleSpinBox(this);
  angle_offset_->setMinimum(0.0);
  angle_offset_->setSingleStep(1.0);
  angle_offset_->setValue(30.0);
  angle_offset_->setDecimals(3);
  auto label = new QLabel("Angle offset (deg)", this);
  label->setToolTip("The angle between the grinder and the media being ground");
  layout->addRow(label, angle_offset_);

  // Radius of the tool
  tool_radius_ = new QDoubleSpinBox(this);
  tool_radius_->setMinimum(0.0);
  tool_radius_->setSingleStep(0.01);
  tool_radius_->setValue(0.1);
  tool_radius_->setDecimals(3);
  layout->addRow(new QLabel("Tool radius (m)", this), tool_radius_);

  // Flip sign
  flip_sign_ = new QCheckBox(this);
  flip_sign_->setToolTip("Flip the sign of the angle offset between rasters (to support snake organization)");
  layout->addRow(new QLabel("Flip", this), flip_sign_);
  setLayout(layout);
}

ToolPathModifier::ConstPtr AngledOrientationToolPathModifierWidget::create() const
{
  return std::make_unique<AngledOrientationModifier>(angle_offset_->value() * M_PI / 180.0, tool_radius_->value(), flip_sign_->checkState());
}

LeadInToolPathModifierWidget::LeadInToolPathModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent)
{
  auto layout = new QFormLayout(this);

  lead_in_angle_ = new QDoubleSpinBox(this);
  lead_in_angle_->setMinimum(0.0);
  lead_in_angle_->setSingleStep(1.0);
  lead_in_angle_->setValue(90.0);
  lead_in_angle_->setDecimals(3);
  auto label = new QLabel("Arc sweep (deg)", this);
  label->setToolTip("How far along the approach trajectory arc the waypoints extend from the first waypoint of the segment");
  layout->addRow(label, lead_in_angle_);

  // Radius of the lead in arc
  lead_in_arc_radius_ = new QDoubleSpinBox(this);
  lead_in_arc_radius_->setMinimum(0.0);
  lead_in_arc_radius_->setSingleStep(0.01);
  lead_in_arc_radius_->setValue(0.1);
  lead_in_arc_radius_->setDecimals(3);
  auto label_rad = new QLabel("Arc radius (m)", this);
  label_rad->setToolTip("Distance from the first segment point to the center of the approach trajectory arc");
  layout->addRow(label_rad, lead_in_arc_radius_);
  // Number of points
  lead_in_num_of_points_ = new QSpinBox(this);
  lead_in_num_of_points_->setMinimum(0.0);
  lead_in_num_of_points_->setSingleStep(1);
  lead_in_num_of_points_->setValue(5);
  auto label_pnt = new QLabel("Lead in number of points", this);
  label_pnt->setToolTip("Number of waypoints along the approach trajectory");
  layout->addRow(label_pnt, lead_in_num_of_points_);

  setLayout(layout);
}

ToolPathModifier::ConstPtr LeadInToolPathModifierWidget::create() const
{
  return std::make_unique<LeadInModifier>(lead_in_angle_->value() * M_PI / 180.0, lead_in_arc_radius_->value(), lead_in_num_of_points_->value());
}

LeadOutToolPathModifierWidget::LeadOutToolPathModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent)
{
  auto layout = new QFormLayout(this);

  // Lead out angle (how steep or shallow the exit of the tool path is)
  lead_out_angle_ = new QDoubleSpinBox(this);
  lead_out_angle_->setMinimum(0.0);
  lead_out_angle_->setSingleStep(1.0);
  lead_out_angle_->setValue(90.0);
  lead_out_angle_->setDecimals(3);
  auto label = new QLabel("Arc sweep (deg)", this);
  label->setToolTip("How far along the exit trajectory arc the waypoints extend from the last waypoint of the segment");
  layout->addRow(label, lead_out_angle_);

  // Radius of the lead out arc
  lead_out_arc_radius_ = new QDoubleSpinBox(this);
  lead_out_arc_radius_->setMinimum(0.0);
  lead_out_arc_radius_->setSingleStep(0.01);
  lead_out_arc_radius_->setValue(0.1);
  lead_out_arc_radius_->setDecimals(3);
  auto label_rad = new QLabel("Arc radius (m)", this);
  label_rad->setToolTip("Distance from the last segment point to the center of the exit trajectory arc");
  layout->addRow(label_rad, lead_out_arc_radius_);

  // Number of points
  lead_out_num_of_points_ = new QSpinBox(this);
  lead_out_num_of_points_->setMinimum(0.0);
  lead_out_num_of_points_->setSingleStep(1);
  lead_out_num_of_points_->setValue(5);

  auto label_pnt = new QLabel("Lead in number of points", this);
  label_pnt->setToolTip("Number of waypoints along the exit trajectory");
  layout->addRow(label_pnt, lead_out_num_of_points_);

  setLayout(layout);
}

ToolPathModifier::ConstPtr LeadOutToolPathModifierWidget::create() const
{
  return std::make_unique<LeadOutModifier>(lead_out_angle_->value() * M_PI / 180.0, lead_out_arc_radius_->value(), lead_out_num_of_points_->value());
}


}  // namespace noether
