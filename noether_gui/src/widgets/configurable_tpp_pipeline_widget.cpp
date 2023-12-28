#include <noether_gui/widgets/configurable_tpp_pipeline_widget.h>
#include <noether_gui/widgets/tpp_pipeline_widget.h>
#include <noether_gui/utils.h>
#include "ui_configurable_tpp_pipeline_widget.h"

#include <fstream>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QStandardPaths>
#include <yaml-cpp/yaml.h>

namespace noether
{
ConfigurableTPPPipelineWidget::ConfigurableTPPPipelineWidget(boost_plugin_loader::PluginLoader loader, QWidget* parent)
  : QWidget(parent)
  , ui_(new Ui::ConfigurableTPPPipeline())
  , pipeline_widget_(new TPPPipelineWidget(std::move(loader), this))
{
  ui_->setupUi(this);
  layout()->addWidget(pipeline_widget_);

  // Connect
  connect(ui_->push_button_load, &QPushButton::clicked, this, &ConfigurableTPPPipelineWidget::onLoadConfiguration);
  connect(ui_->push_button_save, &QPushButton::clicked, this, &ConfigurableTPPPipelineWidget::onSaveConfiguration);
}

ToolPathPlannerPipeline ConfigurableTPPPipelineWidget::createPipeline() const
{
  return pipeline_widget_->createPipeline();
}

void ConfigurableTPPPipelineWidget::configure(const YAML::Node& config) { return pipeline_widget_->configure(config); }

void ConfigurableTPPPipelineWidget::save(YAML::Node& config) const { return pipeline_widget_->save(config); }

void ConfigurableTPPPipelineWidget::setConfigurationFile(const QString& file)
{
  ui_->line_edit->setText(file);

  try
  {
    configure(YAML::LoadFile(file.toStdString()));
  }
  catch (const YAML::BadFile&)
  {
    QString message;
    QTextStream ss;
    ss << "Failed to open YAML file at '" << file << "'";
    QMessageBox::warning(this, "Configuration Error", message);
  }
  catch (const std::exception& ex)
  {
    std::stringstream ss;
    printException(ex, ss);
    QMessageBox::warning(this, "Configuration Error", QString::fromStdString(ss.str()));
  }
}

void ConfigurableTPPPipelineWidget::onLoadConfiguration(const bool /*checked*/)
{
  QString file = ui_->line_edit->text();
  if (file.isEmpty())
    file = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);

  file = QFileDialog::getOpenFileName(this, "Load configuration file", file, "YAML files (*.yaml)");
  if (!file.isEmpty())
    setConfigurationFile(file);
}

void ConfigurableTPPPipelineWidget::onSaveConfiguration(const bool /*checked*/)
{
  try
  {
    QString file = ui_->line_edit->text();
    if (file.isEmpty())
      file = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);

    file = QFileDialog::getSaveFileName(this, "Save configuration file", file, "YAML files (*.yaml)");
    if (file.isEmpty())
      return;

    YAML::Node config;
    save(config);

    std::ofstream ofh(file.toStdString());
    if (!ofh)
      throw std::runtime_error("Failed to open output file at '" + file.toStdString() + "'");

    ofh << config;
    QMessageBox::information(this, "Configuration", "Successfully saved tool path planning pipeline configuration");
  }
  catch (const std::exception& ex)
  {
    std::stringstream ss;
    printException(ex, ss);
    QMessageBox::warning(this, "Save Error", QString::fromStdString(ss.str()));
  }
}

}  // namespace noether
