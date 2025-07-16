#include <noether_gui/widgets/tool_path_modifiers/linear_approach_modifier_widget.h>
#include "../ui_vector3d_editor_widget.h"
#include "ui_linear_approach_modifier_widget.h"

#include <noether_tpp/serialization.h>

static const std::string N_POINTS_KEY = "n_points";
static const std::string AXIS_KEY = "axis";
static const std::string OFFSET_KEY = "offset";

namespace noether
{
LinearApproachToolPathModifierWidget::LinearApproachToolPathModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent), ui_(new Ui::LinearApproachModifier()), vector_editor_ui_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);

  // Add a page to the stacked widget for the axis editor
  auto page_vector = new QWidget(this);
  vector_editor_ui_->setupUi(page_vector);
  vector_editor_ui_->group_box->setTitle("Axis");
  ui_->stackedWidget->addWidget(page_vector);

  // Connect the button to toggle between the axis selector and axis editor
  connect(ui_->combo_box_menu, qOverload<int>(&QComboBox::currentIndexChanged), [this](int index) {
    ui_->stackedWidget->setCurrentIndex(index);
  });

  // Set to z-axis by default
  ui_->combo_box_axis->setCurrentIndex(2);
}

void LinearApproachToolPathModifierWidget::configure(const YAML::Node& config)
{
  ui_->spin_box_points->setValue(YAML::getMember<int>(config, N_POINTS_KEY));
  auto offset = YAML::getMember<Eigen::Vector3d>(config, "offset");

  ui_->combo_box_menu->setCurrentIndex(1);

  vector_editor_ui_->double_spin_box_x->setValue(offset.x());
  vector_editor_ui_->double_spin_box_y->setValue(offset.y());
  vector_editor_ui_->double_spin_box_z->setValue(offset.z());
}

void LinearApproachToolPathModifierWidget::save(YAML::Node& config) const
{
  Eigen::Vector3d vec;
  if (ui_->combo_box_menu->currentIndex() == 0)
  {
    vec = Eigen::Vector3d::Zero();
    vec[ui_->combo_box_axis->currentIndex()] = ui_->double_spin_box_offset->value();
  }
  else
  {
    vec.x() = vector_editor_ui_->double_spin_box_x->value();
    vec.y() = vector_editor_ui_->double_spin_box_y->value();
    vec.z() = vector_editor_ui_->double_spin_box_z->value();
  }

  config["name"] = "LinearApproach";
  config["offset"] = vec;
  config[N_POINTS_KEY] = ui_->spin_box_points->value();
}

}  // namespace noether
