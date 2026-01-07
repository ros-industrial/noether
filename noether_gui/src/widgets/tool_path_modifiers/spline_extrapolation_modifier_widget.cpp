#include <noether_gui/widgets/tool_path_modifiers/spline_extrapolation_modifier_widget.h>
#include "ui_spline_extrapolation_modifier_widget.h"

#include <noether_tpp/serialization.h>

static const char* SPLINE_DEGREE_KEY = "spline_degree";
static const char* EXTRAPOLATION_DISTANCE_FRONT_KEY = "extrapolation_distance_front";
static const char* NORMAL_OFFSET_DISTANCE_FRONT_KEY = "normal_offset_distance_front";
static const char* EXTRAPOLATION_DISTANCE_BACK_KEY = "extrapolation_distance_back";
static const char* NORMAL_OFFSET_DISTANCE_BACK_KEY = "normal_offset_distance_back";

namespace noether
{
SplineExtrapolationToolPathModifierWidget::SplineExtrapolationToolPathModifierWidget(QWidget* parent)
  : BaseWidget(parent), ui_(new Ui::SplineExtrapolation())
{
  ui_->setupUi(this);

  auto configure = [this](int state) {
    switch (state)
    {
      case Qt::CheckState::Checked:
        // Disable the back spin boxes
        ui_->extrapolation_distance_back->setEnabled(false);
        ui_->normal_offset_distance_back->setEnabled(false);

        // Connect the front extrapolation distance spin box to the back
        connect(ui_->extrapolation_distance_front,
                qOverload<double>(&QDoubleSpinBox::valueChanged),
                ui_->extrapolation_distance_back,
                &QDoubleSpinBox::setValue);

        // Connect the front normal offset distance spin box to the back
        connect(ui_->normal_offset_distance_front,
                qOverload<double>(&QDoubleSpinBox::valueChanged),
                ui_->normal_offset_distance_back,
                &QDoubleSpinBox::setValue);

        emit ui_->extrapolation_distance_front->valueChanged(ui_->extrapolation_distance_front->value());
        emit ui_->normal_offset_distance_front->valueChanged(ui_->normal_offset_distance_front->value());

        break;
      default:
        // Enable the back spin boxes
        ui_->extrapolation_distance_back->setEnabled(true);
        ui_->normal_offset_distance_back->setEnabled(true);

        // Disconnect the front extrapolation distance spin box from the back
        disconnect(ui_->extrapolation_distance_front,
                   qOverload<double>(&QDoubleSpinBox::valueChanged),
                   ui_->extrapolation_distance_back,
                   &QDoubleSpinBox::setValue);

        // Disonnect the front normal offset distance spin box from the back
        disconnect(ui_->normal_offset_distance_front,
                   qOverload<double>(&QDoubleSpinBox::valueChanged),
                   ui_->normal_offset_distance_back,
                   &QDoubleSpinBox::setValue);

        break;
    }
  };

  connect(ui_->lock_values, &QCheckBox::stateChanged, configure);

  // Call
  configure(Qt::CheckState::Checked);
}

void SplineExtrapolationToolPathModifierWidget::configure(const YAML::Node& config)
{
  ui_->spline_degree->setValue(YAML::getMember<double>(config, SPLINE_DEGREE_KEY));
  ui_->extrapolation_distance_front->setValue(YAML::getMember<double>(config, EXTRAPOLATION_DISTANCE_FRONT_KEY));
  ui_->normal_offset_distance_front->setValue(YAML::getMember<double>(config, NORMAL_OFFSET_DISTANCE_FRONT_KEY));
  ui_->extrapolation_distance_back->setValue(YAML::getMember<double>(config, EXTRAPOLATION_DISTANCE_BACK_KEY));
  ui_->normal_offset_distance_back->setValue(YAML::getMember<double>(config, NORMAL_OFFSET_DISTANCE_BACK_KEY));
}

void SplineExtrapolationToolPathModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "SplineExtrapolation";
  config[SPLINE_DEGREE_KEY] = ui_->spline_degree->value();
  config[EXTRAPOLATION_DISTANCE_FRONT_KEY] = ui_->extrapolation_distance_front->value();
  config[NORMAL_OFFSET_DISTANCE_FRONT_KEY] = ui_->normal_offset_distance_front->value();
  config[EXTRAPOLATION_DISTANCE_BACK_KEY] = ui_->extrapolation_distance_back->value();
  config[NORMAL_OFFSET_DISTANCE_BACK_KEY] = ui_->normal_offset_distance_back->value();
}

}  // namespace noether
