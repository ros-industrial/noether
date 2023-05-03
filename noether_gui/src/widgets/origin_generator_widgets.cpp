#include <noether_gui/widgets/origin_generator_widgets.h>
#include "ui_vector3d_editor_widget.h"

#include <noether_tpp/tool_path_planners/raster/origin_generators.h>

namespace noether
{
FixedOriginGeneratorWidget::FixedOriginGeneratorWidget(QWidget* parent, const Eigen::Vector3d& dir)
  : OriginGeneratorWidget(parent), ui_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);
  ui_->group_box->setTitle("Origin");

  ui_->double_spin_box_x->setValue(dir.x());
  ui_->double_spin_box_y->setValue(dir.y());
  ui_->double_spin_box_z->setValue(dir.z());
}

OriginGenerator::ConstPtr FixedOriginGeneratorWidget::create() const
{
  Eigen::Vector3d origin(
      ui_->double_spin_box_x->value(), ui_->double_spin_box_y->value(), ui_->double_spin_box_z->value());
  return std::make_unique<FixedOriginGenerator>(origin);
}

OriginGenerator::ConstPtr CentroidOriginGeneratorWidget::create() const
{
  return std::make_unique<CentroidOriginGenerator>();
}

OriginGenerator::ConstPtr AABBOriginGeneratorWidget::create() const
{
  return std::make_unique<AABBCenterOriginGenerator>();
}

}  // namespace noether
