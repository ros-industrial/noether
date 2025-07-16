#include <noether_gui/widgets/tool_path_modifiers/tool_drag_orientation_modifier_widget.h>
#include <noether_gui/widgets/angle_double_spin_box.h>
#include <noether_gui/widgets/distance_double_spin_box.h>

#include <noether_tpp/serialization.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>

static const std::string ANGLE_OFFSET_KEY = "angle_offset";
static const std::string TOOL_RADIUS_KEY = "tool_radius";

namespace noether
{
ToolDragOrientationToolPathModifierWidget::ToolDragOrientationToolPathModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent)
{
  auto layout = new QFormLayout(this);

  // Angle between the grinder and the media being ground
  angle_offset_ = new AngleDoubleSpinBox(this);
  angle_offset_->setMinimum(0.0);
  angle_offset_->setDecimals(3);
  auto label = new QLabel("Angle offset", this);
  label->setToolTip("The angle between the grinder and the media being ground");
  layout->addRow(label, angle_offset_);

  // Radius of the tool
  tool_radius_ = new DistanceDoubleSpinBox(this);
  tool_radius_->setMinimum(0.0);
  tool_radius_->setSingleStep(0.01);
  tool_radius_->setValue(0.1);
  tool_radius_->setDecimals(3);
  layout->addRow(new QLabel("Tool radius", this), tool_radius_);
}

void ToolDragOrientationToolPathModifierWidget::configure(const YAML::Node& config)
{
  angle_offset_->setValue(YAML::getMember<double>(config, ANGLE_OFFSET_KEY));
  tool_radius_->setValue(YAML::getMember<double>(config, TOOL_RADIUS_KEY));
}

void ToolDragOrientationToolPathModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "ToolDragOrientation";
  config[ANGLE_OFFSET_KEY] = angle_offset_->value();
  config[TOOL_RADIUS_KEY] = tool_radius_->value();
}

}  // namespace noether
