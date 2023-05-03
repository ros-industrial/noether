#include <noether_gui/widgets/direction_generator_widgets.h>
#include "ui_vector3d_editor_widget.h"

#include <noether_tpp/tool_path_planners/raster/direction_generators.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QGroupBox>

namespace noether
{
FixedDirectionGeneratorWidget::FixedDirectionGeneratorWidget(QWidget* parent, const Eigen::Vector3d& dir)
  : DirectionGeneratorWidget(parent), ui_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);
  ui_->group_box->setTitle("Direction");

  ui_->double_spin_box_x->setValue(dir.x());
  ui_->double_spin_box_y->setValue(dir.y());
  ui_->double_spin_box_z->setValue(dir.z());
}

DirectionGenerator::ConstPtr FixedDirectionGeneratorWidget::create() const
{
  Eigen::Vector3d dir(
      ui_->double_spin_box_x->value(), ui_->double_spin_box_y->value(), ui_->double_spin_box_z->value());
  dir.normalize();
  return std::make_unique<FixedDirectionGenerator>(dir);
}

PrincipalAxisDirectionGeneratorWidget::PrincipalAxisDirectionGeneratorWidget(QWidget* parent,
                                                                             const double rotation_offset)
  : DirectionGeneratorWidget(parent)
  , layout_(new QFormLayout(this))
  , label_(new QLabel("Rotation offset (deg)", this))
  , rotation_offset_(new QDoubleSpinBox(this))
{
  layout_->addRow(label_, rotation_offset_);
  rotation_offset_->setValue(rotation_offset);
  rotation_offset_->setRange(-180.0, 180.0);
  rotation_offset_->setDecimals(3);
}

DirectionGenerator::ConstPtr PrincipalAxisDirectionGeneratorWidget::create() const
{
  return std::make_unique<PrincipalAxisDirectionGenerator>(rotation_offset_->value() * M_PI / 180.0);
}

}  // namespace noether
