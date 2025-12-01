#include <noether_gui/widgets/configurable_tpp_pipeline_widget.h>
#include <noether_tpp/utils.h>

#include <fstream>
#include <QFileDialog>
#include <QMessageBox>
#include <QSettings>
#include <QTextStream>
#include <yaml-cpp/yaml.h>

namespace noether
{
static const QString SETTINGS_SECTION = QString("ConfigurableTPPPipelineWidget");
const QString ConfigurableTPPPipelineWidget::SETTINGS_KEY_DEFAULT_DIRECTORY = SETTINGS_SECTION + QString("default_"
                                                                                                         "directory");
const QString ConfigurableTPPPipelineWidget::SETTINGS_KEY_LAST_FILE = SETTINGS_SECTION + QString("last_file");

ConfigurableTPPPipelineWidget::ConfigurableTPPPipelineWidget(std::shared_ptr<const WidgetFactory> factory,
                                                             std::string default_configuration_file_directory,
                                                             QWidget* parent)
  : TPPPipelineWidget(factory, parent)
{
  QSettings settings;
  // Set the default configuration file directory once in settings
  settings.setValue(SETTINGS_KEY_DEFAULT_DIRECTORY, QString::fromStdString(default_configuration_file_directory));
  // Add a placeholder entry for the last opened/saved file
  settings.setValue(SETTINGS_KEY_LAST_FILE, QString{});
}

void ConfigurableTPPPipelineWidget::configure(const QString& file)
{
  try
  {
    TPPPipelineWidget::configure(YAML::LoadFile(file.toStdString()));
  }
  catch (const YAML::BadFile&)
  {
    QString message;
    QTextStream ss;
    ss << "Failed to open YAML file at '" << file << "'";
    QMessageBox::warning(this, "Configuration Error", message);
  }

  QSettings settings;
  settings.setValue(SETTINGS_KEY_LAST_FILE, file);
}

void ConfigurableTPPPipelineWidget::onLoadConfiguration(const bool /*checked*/)
{
  QSettings settings;

  QString file = QFileDialog::getOpenFileName(this,
                                              "Load configuration file",
                                              settings.value(SETTINGS_KEY_DEFAULT_DIRECTORY).toString(),
                                              "YAML files (*.yaml)");

  if (!file.isEmpty())
  {
    // Reset the default directory to an empty string such that subsequent file dialogs start in the directory of the
    // last opened file
    settings.setValue(SETTINGS_KEY_DEFAULT_DIRECTORY, QString{});

    // Save the last opened file to settings
    settings.setValue(SETTINGS_KEY_LAST_FILE, file);

    configure(file);
  }
}

void ConfigurableTPPPipelineWidget::onSaveConfiguration(const bool /*checked*/)
{
  try
  {
    QSettings settings;

    QString file = QFileDialog::getSaveFileName(this,
                                                "Save configuration file",
                                                settings.value(SETTINGS_KEY_DEFAULT_DIRECTORY).toString(),
                                                "YAML files (*.yaml)");
    if (file.isEmpty())
      return;

    // Make sure the file has the YAML suffix
    if (!file.endsWith(".yaml"))
      file = file.append(".yaml");

    // Reset the default directory to an empty string such that subsequent file dialogs start in the directory of the
    // last saved file
    settings.setValue(SETTINGS_KEY_DEFAULT_DIRECTORY, QString{});

    // Save the last saved file to settings
    settings.setValue(SETTINGS_KEY_LAST_FILE, file);

    YAML::Node config;
    save(config);

    std::ofstream ofh(file.toStdString());
    if (!ofh)
      throw std::runtime_error("Failed to open output file at '" + file.toStdString() + "'");

    ofh << config;
  }
  catch (const std::exception& ex)
  {
    std::stringstream ss;
    printException(ex, ss);
    QMessageBox::warning(this, "Save Error", QString::fromStdString(ss.str()));
  }
}

}  // namespace noether
