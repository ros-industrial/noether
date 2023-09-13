#include <noether_gui/widgets/tool_path_modifiers/straight_approach_modifier_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_modifiers/straight_approach_modifier.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>

static const std::string OFFSET_HEIGHT_KEY = "offset_height";
static const std::string N_POINTS_KEY = "n_points";

namespace noether
{
StraightApproachToolPathModifierWidget::StraightApproachToolPathModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent)
{
  auto layout = new QFormLayout(this);

  // Height Offset
  offset_height_ = new QDoubleSpinBox(this);
  offset_height_->setMinimum(0.0);
  offset_height_->setSingleStep(0.01);
  offset_height_->setValue(0.1);
  offset_height_->setDecimals(3);
  auto label_offset = new QLabel("Height offset (m)", this);
  label_offset->setToolTip("Distance from the first segment point to the first point of the approach trajectory");
  layout->addRow(label_offset, offset_height_);
  // Number of points
  n_points_ = new QSpinBox(this);
  n_points_->setMinimum(0.0);
  n_points_->setSingleStep(1);
  n_points_->setValue(5);
  auto label_pnt = new QLabel("Lead in number of points", this);
  label_pnt->setToolTip("Number of waypoints along the approach trajectory");
  layout->addRow(label_pnt, n_points_);

  setLayout(layout);
}

ToolPathModifier::ConstPtr StraightApproachToolPathModifierWidget::create() const
{
  return std::make_unique<StraightApproachModifier>(offset_height_->value(), n_points_->value());
}


void StraightApproachToolPathModifierWidget::configure(const YAML::Node& config)
{
  offset_height_->setValue(getEntry<double>(config, OFFSET_HEIGHT_KEY));
  n_points_->setValue(getEntry<int>(config, N_POINTS_KEY));
}

void StraightApproachToolPathModifierWidget::save(YAML::Node& config) const
{
  config[OFFSET_HEIGHT_KEY] = offset_height_->value();
  config[N_POINTS_KEY] = n_points_->value();
}

}  // namespace noether
