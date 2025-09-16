#include <noether_gui/widgets/tool_path_modifiers/circular_lead_out_modifier_widget.h>
#include <noether_gui/widgets/angle_double_spin_box.h>
#include <noether_gui/widgets/distance_double_spin_box.h>

#include <noether_tpp/serialization.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>

static const std::string ARC_ANGLE_KEY = "arc_angle";
static const std::string ARC_RADIUS_KEY = "arc_radius";
static const std::string N_POINTS_KEY = "n_points";

namespace noether
{
CircularLeadOutToolPathModifierWidget::CircularLeadOutToolPathModifierWidget(QWidget* parent) : BaseWidget(parent)
{
  auto layout = new QFormLayout(this);

  // Lead out angle (how steep or shallow the exit of the tool path is)
  arc_angle_ = new AngleDoubleSpinBox(this);
  arc_angle_->setMinimum(0.0);
  arc_angle_->setValue(M_PI_2);
  arc_angle_->setDecimals(3);
  auto label = new QLabel("Arc sweep", this);
  label->setToolTip("How far along the exit trajectory arc the waypoints extend from the last waypoint of the segment");
  layout->addRow(label, arc_angle_);

  // Radius of the lead out arc
  arc_radius_ = new DistanceDoubleSpinBox(this);
  arc_radius_->setMinimum(0.0);
  arc_radius_->setSingleStep(0.01);
  arc_radius_->setValue(0.1);
  arc_radius_->setDecimals(3);
  auto label_rad = new QLabel("Arc radius", this);
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

void CircularLeadOutToolPathModifierWidget::configure(const YAML::Node& config)
{
  arc_angle_->setValue(YAML::getMember<double>(config, ARC_ANGLE_KEY));
  arc_radius_->setValue(YAML::getMember<double>(config, ARC_RADIUS_KEY));
  n_points_->setValue(YAML::getMember<int>(config, N_POINTS_KEY));
}

void CircularLeadOutToolPathModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "CircularLeadOut";
  config[ARC_ANGLE_KEY] = arc_angle_->value();
  config[ARC_RADIUS_KEY] = arc_radius_->value();
  config[N_POINTS_KEY] = n_points_->value();
}

}  // namespace noether
