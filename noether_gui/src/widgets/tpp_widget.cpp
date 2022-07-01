#include <noether_gui/widgets/tpp_widget.h>
#include "ui_tpp_widget.h"
#include <noether_gui/widgets/tpp_pipeline_widget.h>
#include <noether_gui/utils.h>

#include <pcl/io/vtk_lib_io.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>
#include <QTextStream>

namespace noether
{
TPPWidget::TPPWidget(boost_plugin_loader::PluginLoader loader, QWidget* parent) : QWidget(parent), ui_(new Ui::TPP())
{
  ui_->setupUi(this);

  auto pipeline = new TPPPipelineWidget(std::move(loader), this);
  ui_->scroll_area->setWidget(pipeline);

  connect(ui_->push_button_load_mesh, &QPushButton::clicked, this, &TPPWidget::onLoadMesh);
  connect(ui_->push_button_plan, &QPushButton::clicked, this, &TPPWidget::onPlan);
}

std::vector<ToolPaths> TPPWidget::getToolPaths() { return tool_paths_; }

void TPPWidget::onLoadMesh(const bool /*checked*/)
{
  const QString home = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);
  const QString file = QFileDialog::getOpenFileName(this, "Load mesh file", home, "Mesh files (*.ply *.stl)");
  ui_->line_edit_mesh->setText(file);
}

void TPPWidget::onPlan(const bool /*checked*/)
{
  try
  {
    auto pipeline_widget = dynamic_cast<TPPPipelineWidget*>(ui_->scroll_area->widget());
    if (!pipeline_widget)
      throw std::runtime_error("Tool path planning pipeline widget is not configured correctly");

    const std::string mesh_file = ui_->line_edit_mesh->text().toStdString();
    if (mesh_file.empty())
      throw std::runtime_error("No mesh file selected");

    // Load the mesh
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFile(mesh_file, mesh) < 1)
      throw std::runtime_error("Failed to load mesh from file");

    const ToolPathPlannerPipeline pipeline = pipeline_widget->createPipeline();
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
