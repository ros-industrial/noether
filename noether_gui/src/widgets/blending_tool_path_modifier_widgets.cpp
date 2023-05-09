#include <noether_gui/widgets/blending_tool_path_modifier_widgets.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_modifiers/blending_modifiers.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>

static const std::string ANGLE_OFFSET_KEY = "angle_offset";
static const std::string TOOL_RADIUS_KEY = "tool_radius";
static const std::string ARC_ANGLE_KEY = "arc_angle";
static const std::string ARC_RADIUS_KEY = "arc_radius";
static const std::string N_POINTS_KEY = "n_points";

namespace noether
{
ToolDragOrientationToolPathModifierWidget::ToolDragOrientationToolPathModifierWidget(QWidget* parent)
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

ToolPathModifier::ConstPtr ToolDragOrientationToolPathModifierWidget::create() const
{
  return std::make_unique<ToolDragOrientationToolPathModifier>(angle_offset_->value() * M_PI / 180.0,
                                                               tool_radius_->value());
}

void ToolDragOrientationToolPathModifierWidget::configure(const YAML::Node& config)
{
  angle_offset_->setValue(getEntry<double>(config, ANGLE_OFFSET_KEY));
  tool_radius_->setValue(getEntry<double>(config, TOOL_RADIUS_KEY));
}

void ToolDragOrientationToolPathModifierWidget::save(YAML::Node& config) const
{
  config[ANGLE_OFFSET_KEY] = angle_offset_->value();
  config[TOOL_RADIUS_KEY] = tool_radius_->value();
}

CircularLeadInToolPathModifierWidget::CircularLeadInToolPathModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent)
{
  auto layout = new QFormLayout(this);

  arc_angle_ = new QDoubleSpinBox(this);
  arc_angle_->setMinimum(0.0);
  arc_angle_->setSingleStep(1.0);
  arc_angle_->setValue(90.0);
  arc_angle_->setDecimals(3);
  auto label = new QLabel("Arc sweep (deg)", this);
  label->setToolTip("How far along the approach trajectory arc the waypoints extend from the first waypoint of the "
                    "segment");
  layout->addRow(label, arc_angle_);

  // Radius of the lead in arc
  arc_radius_ = new QDoubleSpinBox(this);
  arc_radius_->setMinimum(0.0);
  arc_radius_->setSingleStep(0.01);
  arc_radius_->setValue(0.1);
  arc_radius_->setDecimals(3);
  auto label_rad = new QLabel("Arc radius (m)", this);
  label_rad->setToolTip("Distance from the first segment point to the center of the approach trajectory arc");
  layout->addRow(label_rad, arc_radius_);
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

ToolPathModifier::ConstPtr CircularLeadInToolPathModifierWidget::create() const
{
  return std::make_unique<CircularLeadInModifier>(
      arc_angle_->value() * M_PI / 180.0, arc_radius_->value(), n_points_->value());
}

void CircularLeadInToolPathModifierWidget::configure(const YAML::Node& config)
{
  arc_angle_->setValue(getEntry<double>(config, ARC_ANGLE_KEY));
  arc_radius_->setValue(getEntry<double>(config, ARC_RADIUS_KEY));
  n_points_->setValue(getEntry<int>(config, N_POINTS_KEY));
}

void CircularLeadInToolPathModifierWidget::save(YAML::Node& config) const
{
  config[ARC_ANGLE_KEY] = arc_angle_->value();
  config[ARC_RADIUS_KEY] = arc_radius_->value();
  config[N_POINTS_KEY] = n_points_->value();
}

CircularLeadOutToolPathModifierWidget::CircularLeadOutToolPathModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent)
{
  auto layout = new QFormLayout(this);

  // Lead out angle (how steep or shallow the exit of the tool path is)
  arc_angle_ = new QDoubleSpinBox(this);
  arc_angle_->setMinimum(0.0);
  arc_angle_->setSingleStep(1.0);
  arc_angle_->setValue(90.0);
  arc_angle_->setDecimals(3);
  auto label = new QLabel("Arc sweep (deg)", this);
  label->setToolTip("How far along the exit trajectory arc the waypoints extend from the last waypoint of the segment");
  layout->addRow(label, arc_angle_);

  // Radius of the lead out arc
  arc_radius_ = new QDoubleSpinBox(this);
  arc_radius_->setMinimum(0.0);
  arc_radius_->setSingleStep(0.01);
  arc_radius_->setValue(0.1);
  arc_radius_->setDecimals(3);
  auto label_rad = new QLabel("Arc radius (m)", this);
  label_rad->setToolTip("Distance from the last segment point to the center of the exit trajectory arc");
  layout->addRow(label_rad, arc_radius_);

  // Number of points
  n_points_ = new QSpinBox(this);
  n_points_->setMinimum(0.0);
  n_points_->setSingleStep(1);
  n_points_->setValue(5);

  auto label_pnt = new QLabel("Lead in number of points", this);
  label_pnt->setToolTip("Number of waypoints along the exit trajectory");
  layout->addRow(label_pnt, n_points_);

  setLayout(layout);
}

ToolPathModifier::ConstPtr CircularLeadOutToolPathModifierWidget::create() const
{
  return std::make_unique<CircularLeadOutModifier>(
      arc_angle_->value() * M_PI / 180.0, arc_radius_->value(), n_points_->value());
}

void CircularLeadOutToolPathModifierWidget::configure(const YAML::Node& config)
{
  arc_angle_->setValue(getEntry<double>(config, ARC_ANGLE_KEY));
  arc_radius_->setValue(getEntry<double>(config, ARC_RADIUS_KEY));
  n_points_->setValue(getEntry<int>(config, N_POINTS_KEY));
}

void CircularLeadOutToolPathModifierWidget::save(YAML::Node& config) const
{
  config[ARC_ANGLE_KEY] = arc_angle_->value();
  config[ARC_RADIUS_KEY] = arc_radius_->value();
  config[N_POINTS_KEY] = n_points_->value();
}

}  // namespace noether
