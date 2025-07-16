#include <noether_gui/widgets/tool_path_modifiers/standard_edge_paths_organization_modifier_widget.h>
#include "../ui_vector3d_editor_widget.h"

#include <noether_tpp/serialization.h>

namespace noether
{
StandardEdgePathsOrganizationModifierWidget::StandardEdgePathsOrganizationModifierWidget(QWidget* parent)
  : ToolPathModifierWidget(parent), ui_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);
  ui_->group_box->setTitle("Start Reference");
}

void StandardEdgePathsOrganizationModifierWidget::configure(const YAML::Node& config)
{
  auto ref = YAML::getMember<Eigen::Vector3d>(config, "start_reference");

  ui_->double_spin_box_x->setValue(ref.x());
  ui_->double_spin_box_y->setValue(ref.y());
  ui_->double_spin_box_z->setValue(ref.z());
}

void StandardEdgePathsOrganizationModifierWidget::save(YAML::Node& config) const
{
  Eigen::Vector3d ref(
      ui_->double_spin_box_x->value(), ui_->double_spin_box_y->value(), ui_->double_spin_box_z->value());
  config["name"] = "StandardEdgePathsOrganization";
  config["start_reference"] = ref;
}

}  // namespace noether
