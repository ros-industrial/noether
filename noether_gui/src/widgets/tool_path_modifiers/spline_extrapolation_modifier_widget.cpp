#include <noether_gui/widgets/tool_path_modifiers/spline_extrapolation_modifier_widget.h>
#include "ui_spline_extrapolation_modifier_widget.h"

#include <noether_tpp/serialization.h>

static const char* EXTRAPOLATION_DISTANCE_KEY = "extrapolation_distance";
static const char* NORMAL_OFFSET_DISTANCE_KEY = "normal_offset_distance";
static const char* EXTRAPOLATE_FRONT_KEY = "extrapolate_front";
static const char* EXTRAPOLATE_BACK_KEY = "extrapolate_back";
static const char* SPLINE_DEGREE_KEY = "spline_degree";

namespace noether
{
SplineExtrapolationToolPathModifierWidget::SplineExtrapolationToolPathModifierWidget(QWidget* parent)
  : BaseWidget(parent), ui_(new Ui::SplineExtrapolation())
{
  ui_->setupUi(this);
}

void SplineExtrapolationToolPathModifierWidget::configure(const YAML::Node& config)
{
  ui_->double_spin_box_extrapolation_distance->setValue(YAML::getMember<double>(config, EXTRAPOLATION_DISTANCE_KEY));
  ui_->double_spin_box_normal_offset_distance->setValue(YAML::getMember<double>(config, NORMAL_OFFSET_DISTANCE_KEY));
  ui_->check_box_extrapolate_front->setChecked(YAML::getMember<bool>(config, EXTRAPOLATE_FRONT_KEY));
  ui_->check_box_extrapolate_back->setChecked(YAML::getMember<bool>(config, EXTRAPOLATE_BACK_KEY));
  ui_->spin_box_spline_degree->setValue(YAML::getMember<double>(config, SPLINE_DEGREE_KEY));
}

void SplineExtrapolationToolPathModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "SplineExtrapolation";
  config[EXTRAPOLATION_DISTANCE_KEY] = ui_->double_spin_box_extrapolation_distance->value();
  config[NORMAL_OFFSET_DISTANCE_KEY] = ui_->double_spin_box_normal_offset_distance->value();
  config[EXTRAPOLATE_FRONT_KEY] = ui_->check_box_extrapolate_front->isChecked();
  config[EXTRAPOLATE_BACK_KEY] = ui_->check_box_extrapolate_back->isChecked();
  config[SPLINE_DEGREE_KEY] = ui_->spin_box_spline_degree->value();
}

}  // namespace noether
