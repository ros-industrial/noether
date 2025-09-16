#pragma once

#include <QWidget>
#include <yaml-cpp/yaml.h>

namespace noether
{
/**
 * @ingroup gui_interfaces_widgets
 * @brief Base class for a widget that can configure tool path planning components
 */
class BaseWidget : public QWidget
{
public:
  BaseWidget(QWidget* parent = nullptr) : QWidget(parent) {}

  /**
   * @brief Configures the elements of the widget from a YAML node
   */
  virtual void configure(const YAML::Node&) {}

  /**
   * @brief Saves the configuration of the widget to a YAML node
   */
  virtual void save(YAML::Node&) const = 0;
};

}  // namespace noether
