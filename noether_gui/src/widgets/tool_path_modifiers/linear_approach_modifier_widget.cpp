#include <noether_gui/widgets/tool_path_modifiers/linear_approach_modifier_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_modifiers/linear_approach_modifier.h>
#include "../ui_vector3d_editor_widget.h"
#include "ui_linear_approach_modifier_widget.h"
#include <yaml-cpp/yaml.h>

//! [YAML node keys]
static const std::string N_POINTS_KEY = "n_points";
static const std::string X_KEY = "x";
static const std::string Y_KEY = "y";
static const std::string Z_KEY = "z";
static const std::string AXIS_KEY = "axis";
static const std::string OFFSET_KEY = "offset";
//! [YAML node keys]

namespace noether
{
LinearApproachToolPathModifierWidget::LinearApproachToolPathModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent), ui_(new Ui::LinearApproachModifier()), vector_editor_ui_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);

  //! [Configure GUI]
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
  //! [Configure GUI]
}

//! [Create() method]
ToolPathModifier::ConstPtr LinearApproachToolPathModifierWidget::create() const
{
  if (ui_->combo_box_menu->currentIndex() == 0)
  {
    auto axis = static_cast<LinearApproachModifier::Axis>(ui_->combo_box_axis->currentIndex());
    return std::make_unique<LinearApproachModifier>(
        ui_->double_spin_box_offset->value(), axis, ui_->spin_box_points->value());
  }
  else
  {
    Eigen::Vector3d dir(vector_editor_ui_->double_spin_box_x->value(),
                        vector_editor_ui_->double_spin_box_y->value(),
                        vector_editor_ui_->double_spin_box_z->value());
    return std::make_unique<LinearApproachModifier>(dir, ui_->spin_box_points->value());
  }
}
//! [Create() method]

//! [Configure() method]
void LinearApproachToolPathModifierWidget::configure(const YAML::Node& config)
{
  ui_->spin_box_points->setValue(getEntry<int>(config, N_POINTS_KEY));
  if (config[OFFSET_KEY].IsDefined() && config[AXIS_KEY])
  {
    ui_->double_spin_box_offset->setValue(getEntry<double>(config, OFFSET_KEY));
    ui_->combo_box_axis->setCurrentIndex(getEntry<int>(config, AXIS_KEY));
    ui_->combo_box_menu->setCurrentIndex(0);
  }
  else if (config[X_KEY].IsDefined() && config[Y_KEY].IsDefined() && config[Z_KEY].IsDefined())
  {
    ui_->combo_box_menu->setCurrentIndex(1);

    vector_editor_ui_->double_spin_box_x->setValue(getEntry<double>(config, X_KEY));
    vector_editor_ui_->double_spin_box_y->setValue(getEntry<double>(config, Y_KEY));
    vector_editor_ui_->double_spin_box_z->setValue(getEntry<double>(config, Z_KEY));
  }
}
//! [Configure() method]

//! [Save() method]
void LinearApproachToolPathModifierWidget::save(YAML::Node& config) const
{
  if (ui_->combo_box_menu->currentIndex() == 0)
  {
    config[AXIS_KEY] = ui_->combo_box_axis->currentIndex();
    config[OFFSET_KEY] = ui_->double_spin_box_offset->value();
  }
  else
  {
    config[X_KEY] = vector_editor_ui_->double_spin_box_x->value();
    config[Y_KEY] = vector_editor_ui_->double_spin_box_y->value();
    config[Z_KEY] = vector_editor_ui_->double_spin_box_z->value();
  }

  config[N_POINTS_KEY] = ui_->spin_box_points->value();
}
//! [Save() method]


}  // namespace noether
