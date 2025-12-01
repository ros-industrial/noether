#include <noether_gui/widgets/mesh_modifiers/ransac_primitive_fit_modifier_widget.h>
#include <noether_gui/widgets/angle_double_spin_box.h>
#include <noether_gui/widgets/distance_double_spin_box.h>
#include "../ui_vector3d_editor_widget.h"
#include "ui_ransac_primitive_fit_modifier_widget.h"

#include <noether_tpp/serialization.h>

static const std::string DIST_THRESH_KEY = "distance_threshold";
static const std::string MIN_VERTICES_KEY = "min_vertices";
static const std::string MAX_PRIMITIVES_KEY = "max_primitives";
static const std::string MAX_ITERATIONS_KEY = "max_iterations";

namespace noether
{
RansacPrimitiveFitMeshModifierWidget::RansacPrimitiveFitMeshModifierWidget(QWidget* parent)
  : ui_(new Ui::RansacPrimitiveFit()), axis_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);
}

void RansacPrimitiveFitMeshModifierWidget::configure(const YAML::Node& config)
{
  ui_->distance_threshold->setValue(YAML::getMember<double>(config, DIST_THRESH_KEY));
  ui_->min_vertices->setValue(YAML::getMember<int>(config, MIN_VERTICES_KEY));
  ui_->max_primitives->setValue(YAML::getMember<int>(config, MAX_PRIMITIVES_KEY));
  ui_->max_iterations->setValue(YAML::getMember<int>(config, MAX_ITERATIONS_KEY));
}

void RansacPrimitiveFitMeshModifierWidget::save(YAML::Node& config) const
{
  config[DIST_THRESH_KEY] = ui_->distance_threshold->value();
  config[MIN_VERTICES_KEY] = ui_->min_vertices->value();
  config[MAX_PRIMITIVES_KEY] = ui_->max_primitives->value();
  config[MAX_ITERATIONS_KEY] = ui_->max_iterations->value();
}

}  // namespace noether
