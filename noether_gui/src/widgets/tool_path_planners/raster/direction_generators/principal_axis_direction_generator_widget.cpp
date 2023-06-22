#include <noether_gui/widgets/tool_path_planners/raster/direction_generators/principal_axis_direction_generator_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_planners/raster/direction_generators/principal_axis_direction_generator.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <yaml-cpp/yaml.h>

namespace noether
{
PrincipalAxisDirectionGeneratorWidget::PrincipalAxisDirectionGeneratorWidget(QWidget* parent)
  : DirectionGeneratorWidget(parent)
  , layout_(new QFormLayout(this))
  , label_(new QLabel("Rotation offset (deg)", this))
  , rotation_offset_(new QDoubleSpinBox(this))
{
  layout_->addRow(label_, rotation_offset_);
  rotation_offset_->setValue(0.0);
  rotation_offset_->setRange(-180.0, 180.0);
  rotation_offset_->setDecimals(3);
}

void PrincipalAxisDirectionGeneratorWidget::configure(const YAML::Node& config)
{
  rotation_offset_->setValue(getEntry<double>(config, "rotation_offset"));
}

void PrincipalAxisDirectionGeneratorWidget::save(YAML::Node& config) const
{
  config["rotation_offset"] = rotation_offset_->value();
}

DirectionGenerator::ConstPtr PrincipalAxisDirectionGeneratorWidget::create() const
{
  return std::make_unique<PrincipalAxisDirectionGenerator>(rotation_offset_->value() * M_PI / 180.0);
}

}  // namespace noether
