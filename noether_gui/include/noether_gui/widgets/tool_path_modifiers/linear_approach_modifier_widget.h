#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_modifier.h>

namespace Ui
{
class Vector3dEditor;
class LinearApproachModifier;
}  // namespace Ui

namespace noether
{
class LinearApproachToolPathModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  LinearApproachToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  Ui::LinearApproachModifier* ui_;
  Ui::Vector3dEditor* vector_editor_ui_;
};

}  // namespace noether
