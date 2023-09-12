#include <noether_gui/widgets/tool_path_modifiers/linear_approach_modifier_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_modifiers/linear_approach_modifier.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include "ui_vector3d_editor_widget.h"


static const std::string OFFSET_KEY = "offset";
static const std::string N_POINTS_KEY = "n_points";

namespace noether
{
LinearApproachToolPathModifierWidget::LinearApproachToolPathModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent), ui_(new Ui::Vector3dEditor())
{
//  auto layout = new QFormLayout(this);
//  // Height Offset
////  offset_ = new QDoubleSpinBox(this);
////  offset_->setMinimum(0.0);
////  offset_->setSingleStep(0.01);
////  offset_->setValue(0.1);
////  offset_->setDecimals(3);
////  auto label_offset = new QLabel("Height offset (m)", this);
////  label_offset->setToolTip("Distance from the first segment point to the first point of the approach trajectory");
////  layout->addRow(label_offset, offset_);
//  ui_->setupUi(this);
//  ui_->group_box->setTitle("Offset (m)");

//  ui_->double_spin_box_z->setValue(0.1);

//  // Number of points
//  n_points_ = new QSpinBox(this);
//  n_points_->setMinimum(0.0);
//  n_points_->setSingleStep(1);
//  n_points_->setValue(5);
//  auto label_pnt = new QLabel("Lead in number of points", this);
//  label_pnt->setToolTip("Number of waypoints along the approach trajectory");
//  layout->addRow(label_pnt, n_points_);

//  setLayout(layout);
  // Create a vertical layout for the entire widget
  auto mainLayout = new QVBoxLayout(this);

  ui_->setupUi(this);
  ui_->group_box->setTitle("Offset (m)");
  ui_->double_spin_box_z->setValue(0.1);

  // Create a container widget to hold the layouts
  auto containerWidget = new QWidget;

  // Create a layout for the container widget
  auto containerLayout = new QVBoxLayout(containerWidget);

  // Create a horizontal layout for the spin box and label
  auto spinBoxLayout = new QHBoxLayout;

  // Number of points
  n_points_ = new QSpinBox(this);
  n_points_->setMinimum(0);
  n_points_->setSingleStep(1);
  n_points_->setValue(5);

  auto label_pnt = new QLabel("Lead in number of points", this);
  label_pnt->setToolTip("Number of waypoints along the approach trajectory");

  // Add label and spin box to the horizontal layout
  spinBoxLayout->addWidget(label_pnt);
  spinBoxLayout->addWidget(n_points_);

  // Add the horizontal layout to the container layout
  containerLayout->addLayout(spinBoxLayout);

  // Set the container widget as the layout for the Vector3dEditor widget
  ui_->group_box->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  containerLayout->addWidget(ui_->group_box);

  // Add the container widget to the main layout
  mainLayout->addWidget(containerWidget);

  // Set the main layout for the entire widget
  setLayout(mainLayout);
}

ToolPathModifier::ConstPtr LinearApproachToolPathModifierWidget::create() const
{
  Eigen::Vector3d dir(
      ui_->double_spin_box_x->value(), ui_->double_spin_box_y->value(), ui_->double_spin_box_z->value());
  return std::make_unique<LinearApproachModifier>(dir, n_points_->value());
}


void LinearApproachToolPathModifierWidget::configure(const YAML::Node& config)
{
//  offset_->setValue(getEntry<Eigen::Vector3d>(config, OFFSET_KEY));
  Eigen::Vector3d dir(getEntry<double>(config, "x"), getEntry<double>(config, "y"), getEntry<double>(config, "z"));

  ui_->double_spin_box_x->setValue(dir.x());
  ui_->double_spin_box_y->setValue(dir.y());
  ui_->double_spin_box_z->setValue(dir.z());
  n_points_->setValue(getEntry<int>(config, N_POINTS_KEY));
}

void LinearApproachToolPathModifierWidget::save(YAML::Node& config) const
{
//  config[OFFSET_KEY] = offset_->value();
  config["x"] = ui_->double_spin_box_x->value();
  config["y"] = ui_->double_spin_box_y->value();
  config["z"] = ui_->double_spin_box_z->value();
  config[N_POINTS_KEY] = n_points_->value();
}

}  // namespace noether
