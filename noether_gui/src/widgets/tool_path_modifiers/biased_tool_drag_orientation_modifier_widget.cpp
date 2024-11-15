#include <noether_gui/widgets/tool_path_modifiers/biased_tool_drag_orientation_modifier_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_modifiers/biased_tool_drag_orientation_modifier.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>

static const std::string ANGLE_OFFSET_KEY = "angle_offset";
static const std::string TOOL_RADIUS_KEY = "tool_radius";

namespace noether
{
BiasedToolDragOrientationToolPathModifierWidget::BiasedToolDragOrientationToolPathModifierWidget(QWidget* parent)
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
}

ToolPathModifier::ConstPtr BiasedToolDragOrientationToolPathModifierWidget::create() const
{
  return std::make_unique<BiasedToolDragOrientationToolPathModifier>(angle_offset_->value() * M_PI / 180.0,
                                                                     tool_radius_->value());
}

void BiasedToolDragOrientationToolPathModifierWidget::configure(const YAML::Node& config)
{
  angle_offset_->setValue(getEntry<double>(config, ANGLE_OFFSET_KEY));
  tool_radius_->setValue(getEntry<double>(config, TOOL_RADIUS_KEY));
}

void BiasedToolDragOrientationToolPathModifierWidget::save(YAML::Node& config) const
{
  config[ANGLE_OFFSET_KEY] = angle_offset_->value();
  config[TOOL_RADIUS_KEY] = tool_radius_->value();
}

}  // namespace noether
