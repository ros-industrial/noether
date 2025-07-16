#pragma once

#include <noether_gui/widgets.h>

namespace Ui
{
class Vector3dEditor;
}

namespace noether
{
class FixedOriginGeneratorWidget : public OriginGeneratorWidget
{
public:
  FixedOriginGeneratorWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  Ui::Vector3dEditor* ui_;
};

}  // namespace noether
