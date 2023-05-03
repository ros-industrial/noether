#include <noether_gui/widgets/tpp_widget.h>
#include "ui_tpp_widget.h"
#include <noether_gui/widgets/tpp_pipeline_widget.h>
#include <noether_gui/utils.h>

#include <fstream>
#include <pcl/io/vtk_lib_io.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>
#include <QTextStream>
#include <yaml-cpp/yaml.h>

namespace noether
{
TPPWidget::TPPWidget(boost_plugin_loader::PluginLoader loader, QWidget* parent)
  : QWidget(parent), ui_(new Ui::TPP()), pipeline_widget_(new TPPPipelineWidget(std::move(loader), this))
{
  ui_->setupUi(this);

  ui_->scroll_area->setWidget(pipeline_widget_);

  connect(ui_->push_button_load_mesh, &QPushButton::clicked, this, &TPPWidget::onLoadMesh);
  connect(ui_->push_button_load_configuration, &QPushButton::clicked, this, &TPPWidget::onLoadConfiguration);
  connect(ui_->push_button_save_configuration, &QPushButton::clicked, this, &TPPWidget::onSaveConfiguration);
  connect(ui_->push_button_plan, &QPushButton::clicked, this, &TPPWidget::onPlan);
}

std::vector<ToolPaths> TPPWidget::getToolPaths() { return tool_paths_; }

void TPPWidget::onLoadMesh(const bool /*checked*/)
{
  const QString home = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);
  const QString file = QFileDialog::getOpenFileName(this, "Load mesh file", home, "Mesh files (*.ply *.stl)");
  ui_->line_edit_mesh->setText(file);
}

void TPPWidget::onLoadConfiguration(const bool /*checked*/)
{
  QString file = ui_->line_edit_configuration->text();
  if (file.isEmpty())
    file = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);

  file = QFileDialog::getOpenFileName(this, "Load configuration file", file, "YAML files (*.yaml)");
  if (file.isEmpty())
    return;

  ui_->line_edit_configuration->setText(file);

  try
  {
    pipeline_widget_->configure(YAML::LoadFile(file.toStdString()));
    QMessageBox::information(this, "Configuration", "Successfully loaded tool path planning pipeline configuration");
  }
  catch (const YAML::BadFile& ex)
  {
    QString message;
    QTextStream ss(&message);
    ss << "Failed to open YAML file at '" << file << "'";
    QMessageBox::warning(this, "Configuration Error", message);
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Configuration Error", ex.what());
  }
}

void TPPWidget::onSaveConfiguration(const bool /*checked*/)
{
  QString file = ui_->line_edit_configuration->text();
  if (file.isEmpty())
    file = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);

  file = QFileDialog::getSaveFileName(this, "Save configuration file", file, "YAML files (*.yaml)");
  if (file.isEmpty())
    return;

  YAML::Node config;
  pipeline_widget_->save(config);

  std::ofstream ofh(file.toStdString());
  if (!ofh)
  {
    QString message;
    QTextStream ss(&message);
    ss << "Failed to open output file at '" << file << "'";
    QMessageBox::warning(this, "Error", message);
  }
  else
  {
    ofh << config;
    QMessageBox::information(this, "Configuration", "Successfully saved tool path planning pipeline configuration");
  }
}

void TPPWidget::onPlan(const bool /*checked*/)
{
  try
  {
    const std::string mesh_file = ui_->line_edit_mesh->text().toStdString();
    if (mesh_file.empty())
      throw std::runtime_error("No mesh file selected");

    // Load the mesh
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFile(mesh_file, mesh) < 1)
      throw std::runtime_error("Failed to load mesh from file");

    const ToolPathPlannerPipeline pipeline = pipeline_widget_->createPipeline();
    QApplication::setOverrideCursor(Qt::WaitCursor);
    tool_paths_ = pipeline.plan(mesh);
    QApplication::restoreOverrideCursor();

    QString message;
    QTextStream ss(&message);
    for (std::size_t i = 0; i < tool_paths_.size(); ++i)
    {
      ss << "Mesh fragment " << i + 1 << ": created tool path with " << tool_paths_[i].size() << " strokes\n";
    }
    QMessageBox::information(this, "Success", message);
  }
  catch (const std::exception& ex)
  {
    QApplication::restoreOverrideCursor();
    QMessageBox::warning(this, "Tool Path Planning Error", QString::fromStdString(ex.what()));
  }
}

}  // namespace noether
