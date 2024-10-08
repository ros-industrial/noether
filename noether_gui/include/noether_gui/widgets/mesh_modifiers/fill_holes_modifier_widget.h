#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/mesh_modifier.h>

class QDoubleSpinBox;

namespace noether
{
class FillHolesModifierWidget : public MeshModifierWidget
{
public:
  FillHolesModifierWidget(QWidget* parent = nullptr);

  MeshModifier::ConstPtr create() const override;
  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  QDoubleSpinBox* max_hole_size_;
};

}  // namespace noether
