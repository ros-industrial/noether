#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace Ui
{
class Vector3dEditor;
}

namespace noether
{
class FixedOriginGeneratorWidget : public OriginGeneratorWidget
{
  Q_OBJECT
public:
  FixedOriginGeneratorWidget(QWidget* parent = nullptr);

  OriginGenerator::ConstPtr create() const override;

  void fromYAML(const YAML::Node&) override;

private:
  Ui::Vector3dEditor* ui_;
};

struct CentroidOriginGeneratorWidget : public OriginGeneratorWidget
{
  Q_OBJECT
public:
  using BaseWidget::BaseWidget;

  OriginGenerator::ConstPtr create() const override;
};

struct AABBOriginGeneratorWidget : public OriginGeneratorWidget
{
  Q_OBJECT
public:
  using BaseWidget::BaseWidget;

  OriginGenerator::ConstPtr create() const override;
};

}  // namespace noether
