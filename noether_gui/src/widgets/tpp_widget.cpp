#include <noether_gui/widgets/tpp_widget.h>
#include "ui_tpp_widget.h"
#include <noether_gui/widgets/tpp_pipeline_widget.h>
#include <noether_gui/utils.h>

#include <fstream>
#include <pcl/io/vtk_lib_io.h>
#include <QColorDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>
#include <QTextStream>
#include <yaml-cpp/yaml.h>

// Rendering includes
#include <QVTKWidget.h>
#include <vtkAxesActor.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLActor.h>
#include <vtkOpenGLRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkPLYReader.h>
#include <vtkSTLReader.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAxes.h>
#include <vtkTransformFilter.h>
#include <vtkTransform.h>
#include <vtkTubeFilter.h>

namespace noether
{
TPPWidget::TPPWidget(boost_plugin_loader::PluginLoader loader, QWidget* parent)
  : QWidget(parent)
  , ui_(new Ui::TPP())
  , pipeline_widget_(new TPPPipelineWidget(std::move(loader), this))
  , render_widget_(new QVTKWidget(this))
  , renderer_(vtkOpenGLRenderer::New())
  , mesh_mapper_(vtkOpenGLPolyDataMapper::New())
  , mesh_actor_(vtkOpenGLActor::New())
  , axes_(vtkAxes::New())
  , tube_filter_(vtkTubeFilter::New())
{
  ui_->setupUi(this);

  ui_->scroll_area->setWidget(pipeline_widget_);
  ui_->splitter->addWidget(render_widget_);

  // Set up the VTK objects
  mesh_actor_->SetMapper(mesh_mapper_);
  renderer_->AddActor(mesh_actor_);
  renderer_->SetBackground(0.2, 0.2, 0.2);
  axes_->SetScaleFactor(ui_->double_spin_box_axis_size->value());
  tube_filter_->SetInputConnection(axes_->GetOutputPort());
  tube_filter_->SetRadius(axes_->GetScaleFactor() / 10.0);
  tube_filter_->SetNumberOfSides(10);
  tube_filter_->CappingOn();

  vtkRenderWindow* window = render_widget_->GetRenderWindow();
  window->AddRenderer(renderer_);
  render_widget_->GetInteractor()->SetInteractorStyle(vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New());
  render_widget_->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

  // Connect signals
  connect(ui_->push_button_load_mesh, &QPushButton::clicked, this, &TPPWidget::onLoadMesh);
  connect(ui_->push_button_load_configuration, &QPushButton::clicked, this, &TPPWidget::onLoadConfiguration);
  connect(ui_->push_button_save_configuration, &QPushButton::clicked, this, &TPPWidget::onSaveConfiguration);
  connect(ui_->push_button_plan, &QPushButton::clicked, this, &TPPWidget::onPlan);
  connect(ui_->double_spin_box_axis_size, &QDoubleSpinBox::editingFinished, this, [this]() {
    axes_->SetScaleFactor(ui_->double_spin_box_axis_size->value());
    tube_filter_->SetRadius(axes_->GetScaleFactor() / 10.0);
    render_widget_->GetRenderWindow()->Render();
    render_widget_->GetRenderWindow()->Render();
  });
}

TPPWidget::~TPPWidget()
{
  renderer_->Delete();
  mesh_mapper_->Delete();
  mesh_actor_->Delete();
  axes_->Delete();
  tube_filter_->Delete();
}

std::vector<ToolPaths> TPPWidget::getToolPaths() { return tool_paths_; }

void TPPWidget::setMeshFile(const QString& file)
{
  ui_->line_edit_mesh->setText(file);

  // Render the mesh
  vtkSmartPointer<vtkAbstractPolyDataReader> reader;
  if (file.endsWith(".ply"))
    reader = vtkSmartPointer<vtkPLYReader>::New();
  else if (file.endsWith(".stl"))
    reader = vtkSmartPointer<vtkSTLReader>::New();
  else
    return;

  reader->SetFileName(file.toStdString().c_str());
  reader->Update();

  // Update the mapper input data
  mesh_mapper_->SetInputData(reader->GetOutput());

  // Zoom out to the extents
  renderer_->ResetCamera();

  // Call render twice
  render_widget_->GetRenderWindow()->Render();
  render_widget_->GetRenderWindow()->Render();
}

void TPPWidget::onLoadMesh(const bool /*checked*/)
{
  const QString home = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);
  const QString file = QFileDialog::getOpenFileName(this, "Load mesh file", home, "Mesh files (*.ply *.stl)");
  if (!file.isNull())
    setMeshFile(file);
}

void TPPWidget::setConfigurationFile(const QString& file)
{
  ui_->line_edit_configuration->setText(file);

  try
  {
    pipeline_widget_->configure(YAML::LoadFile(file.toStdString()));
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

void TPPWidget::onLoadConfiguration(const bool /*checked*/)
{
  QString file = ui_->line_edit_configuration->text();
  if (file.isEmpty())
    file = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);

  file = QFileDialog::getOpenFileName(this, "Load configuration file", file, "YAML files (*.yaml)");
  if (!file.isEmpty())
    setConfigurationFile(file);
}

void TPPWidget::onSaveConfiguration(const bool /*checked*/)
{
  try
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

vtkSmartPointer<vtkTransform> toVTK(const Eigen::Isometry3d& mat)
{
  auto t = vtkSmartPointer<vtkTransform>::New();
  t->Translate(mat.translation().data());
  Eigen::AngleAxisd aa(mat.rotation());
  t->RotateWXYZ(aa.angle() * 180.0 / M_PI, aa.axis().data());
  return t;
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

    // Render the tool paths
    std::for_each(tool_path_actors_.begin(), tool_path_actors_.end(), [this](vtkProp* actor) {
      renderer_->RemoveActor(actor);
      actor->Delete();
    });
    tool_path_actors_.clear();

    for (const ToolPaths& fragment : tool_paths_)
    {
      for (const ToolPath& tool_path : fragment)
      {
        for (const ToolPathSegment& segment : tool_path)
        {
          for (const Eigen::Isometry3d& w : segment)
          {
            auto transform_filter = vtkSmartPointer<vtkTransformFilter>::New();
            transform_filter->SetTransform(toVTK(w));
            transform_filter->SetInputConnection(tube_filter_->GetOutputPort());

            auto map = vtkSmartPointer<vtkPolyDataMapper>::New();
            map->SetInputConnection(transform_filter->GetOutputPort());

            auto actor = vtkActor::New();
            actor->SetMapper(map);

            tool_path_actors_.push_back(actor);
          }
        }
      }
    }

    std::for_each(
        tool_path_actors_.begin(), tool_path_actors_.end(), [this](vtkProp* actor) { renderer_->AddActor(actor); });

    // Call render twice
    render_widget_->GetRenderWindow()->Render();
    render_widget_->GetRenderWindow()->Render();
  }
  catch (const std::exception& ex)
  {
    QApplication::restoreOverrideCursor();

    std::stringstream ss;
    printException(ex, ss);
    QMessageBox::warning(this, "Tool Path Planning Error", QString::fromStdString(ss.str()));
  }
}

}  // namespace noether
