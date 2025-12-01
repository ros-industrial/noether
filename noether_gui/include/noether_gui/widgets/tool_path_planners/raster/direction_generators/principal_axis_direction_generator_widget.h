#pragma once

#include <noether_gui/widgets.h>

class QFormLayout;
class QLabel;

namespace noether
{
class AngleDoubleSpinBox;

class PrincipalAxisDirectionGeneratorWidget : public BaseWidget
{
public:
  PrincipalAxisDirectionGeneratorWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QFormLayout* layout_;
  QLabel* label_;
  AngleDoubleSpinBox* rotation_offset_;
};

}  // namespace noether
