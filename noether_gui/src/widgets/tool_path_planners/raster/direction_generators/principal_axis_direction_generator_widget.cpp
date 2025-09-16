#include <noether_gui/widgets/tool_path_planners/raster/direction_generators/principal_axis_direction_generator_widget.h>
#include <noether_gui/widgets/angle_double_spin_box.h>
#include <noether_gui/utils.h>

#include <noether_tpp/serialization.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>

namespace noether
{
PrincipalAxisDirectionGeneratorWidget::PrincipalAxisDirectionGeneratorWidget(QWidget* parent)
  : BaseWidget(parent)
  , layout_(new QFormLayout(this))
  , label_(new QLabel("Rotation offset", this))
  , rotation_offset_(new AngleDoubleSpinBox(this))
{
  layout_->addRow(label_, rotation_offset_);
  rotation_offset_->setValue(M_PI_2);
  rotation_offset_->setDecimals(3);
}

void PrincipalAxisDirectionGeneratorWidget::configure(const YAML::Node& config)
{
  auto rotation_offset = YAML::getMember<double>(config, "rotation_offset");
  rotation_offset_->setValue(rotation_offset);
}

void PrincipalAxisDirectionGeneratorWidget::save(YAML::Node& config) const
{
  config["name"] = "PrincipalAxis";
  config["rotation_offset"] = rotation_offset_->value();
}

}  // namespace noether
