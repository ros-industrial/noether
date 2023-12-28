#include <noether_gui/widgets/tpp_widget.h>
#include "ui_tpp_widget.h"
#include <noether_gui/widgets/configurable_tpp_pipeline_widget.h>
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
#include <vtkAssembly.h>
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
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

namespace noether
{
TPPWidget::TPPWidget(boost_plugin_loader::PluginLoader loader, QWidget* parent)
  : QWidget(parent)
  , ui_(new Ui::TPP())
  , pipeline_widget_(new ConfigurableTPPPipelineWidget(std::move(loader), this))
  , render_widget_(new QVTKWidget(this))
  , renderer_(vtkSmartPointer<vtkOpenGLRenderer>::New())
  , mesh_mapper_(vtkSmartPointer<vtkOpenGLPolyDataMapper>::New())
  , mesh_actor_(vtkSmartPointer<vtkOpenGLActor>::New())
  , mesh_fragment_actor_(vtkSmartPointer<vtkAssembly>::New())
  , tool_path_actor_(vtkSmartPointer<vtkAssembly>::New())
  , unmodified_tool_path_actor_(vtkSmartPointer<vtkAssembly>::New())
  , axes_(vtkSmartPointer<vtkAxes>::New())
  , tube_filter_(vtkSmartPointer<vtkTubeFilter>::New())
{
  ui_->setupUi(this);

  overwriteWidget(ui_->group_box_configuration->layout(), ui_->widget, pipeline_widget_);
  ui_->splitter->addWidget(render_widget_);

  // Set up the VTK objects
  renderer_->SetBackground(0.2, 0.2, 0.2);

  // Original mesh mapper/actor
  mesh_actor_->SetMapper(mesh_mapper_);
  renderer_->AddActor(mesh_actor_);

  // Mesh fragment mapper/actor
  renderer_->AddActor(mesh_fragment_actor_);

  // Tool path axis display object
  axes_->SetScaleFactor(ui_->double_spin_box_axis_size->value());
  tube_filter_->SetInputConnection(axes_->GetOutputPort());
  tube_filter_->SetRadius(axes_->GetScaleFactor() / 10.0);
  tube_filter_->SetNumberOfSides(10);
  tube_filter_->CappingOn();

  vtkRenderWindow* window = render_widget_->GetRenderWindow();
  window->AddRenderer(renderer_);
  render_widget_->GetInteractor()->SetInteractorStyle(vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New());
  render_widget_->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

  // Set visibility of the actors based on the default state of the check boxes
  mesh_actor_->SetVisibility(ui_->check_box_show_original_mesh->isChecked());
  mesh_fragment_actor_->SetVisibility(ui_->check_box_show_modified_mesh->isChecked());
  unmodified_tool_path_actor_->SetVisibility(ui_->check_box_show_original_tool_path->isChecked());
  tool_path_actor_->SetVisibility(ui_->check_box_show_modified_tool_path->isChecked());

  // Connect signals
  connect(ui_->push_button_load_mesh, &QPushButton::clicked, this, &TPPWidget::onLoadMesh);
  connect(ui_->check_box_show_original_mesh, &QCheckBox::clicked, this, &TPPWidget::onShowOriginalMesh);
  connect(ui_->check_box_show_modified_mesh, &QCheckBox::clicked, this, &TPPWidget::onShowModifiedMesh);
  connect(ui_->check_box_show_original_tool_path, &QCheckBox::clicked, this, &TPPWidget::onShowUnmodifiedToolPath);
  connect(ui_->check_box_show_modified_tool_path, &QCheckBox::clicked, this, &TPPWidget::onShowModifiedToolPath);
  connect(ui_->push_button_plan, &QPushButton::clicked, this, &TPPWidget::onPlan);
  connect(ui_->double_spin_box_axis_size, &QDoubleSpinBox::editingFinished, this, [this]() {
    axes_->SetScaleFactor(ui_->double_spin_box_axis_size->value());
    tube_filter_->SetRadius(axes_->GetScaleFactor() / 10.0);
    render_widget_->GetRenderWindow()->Render();
    render_widget_->GetRenderWindow()->Render();
  });
}

void TPPWidget::onShowOriginalMesh(const bool checked)
{
  mesh_actor_->SetVisibility(checked);
  render_widget_->GetRenderWindow()->Render();
  render_widget_->GetRenderWindow()->Render();
}

void TPPWidget::onShowModifiedMesh(const bool checked)
{
  mesh_fragment_actor_->SetVisibility(checked);
  render_widget_->GetRenderWindow()->Render();
  render_widget_->GetRenderWindow()->Render();
}

void TPPWidget::onShowUnmodifiedToolPath(const bool checked)
{
  unmodified_tool_path_actor_->SetVisibility(checked);
  render_widget_->GetRenderWindow()->Render();
  render_widget_->GetRenderWindow()->Render();
}

void TPPWidget::onShowModifiedToolPath(const bool checked)
{
  tool_path_actor_->SetVisibility(checked);
  render_widget_->GetRenderWindow()->Render();
  render_widget_->GetRenderWindow()->Render();
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

void TPPWidget::setConfigurationFile(const QString& file) { pipeline_widget_->setConfigurationFile(file); }

vtkSmartPointer<vtkTransform> toVTK(const Eigen::Isometry3d& mat)
{
  auto t = vtkSmartPointer<vtkTransform>::New();
  t->Translate(mat.translation().data());
  Eigen::AngleAxisd aa(mat.rotation());
  t->RotateWXYZ(aa.angle() * 180.0 / M_PI, aa.axis().data());
  return t;
}

vtkSmartPointer<vtkAssembly> createToolPathActors(const std::vector<ToolPaths>& tool_paths,
                                                  vtkAlgorithmOutput* waypoint_shape_output_port)
{
  auto assembly = vtkSmartPointer<vtkAssembly>::New();
  for (const ToolPaths& fragment : tool_paths)
  {
    for (const ToolPath& tool_path : fragment)
    {
      for (const ToolPathSegment& segment : tool_path)
      {
        for (const Eigen::Isometry3d& w : segment)
        {
          auto transform_filter = vtkSmartPointer<vtkTransformFilter>::New();
          transform_filter->SetTransform(toVTK(w));
          transform_filter->SetInputConnection(waypoint_shape_output_port);

          auto map = vtkSmartPointer<vtkPolyDataMapper>::New();
          map->SetInputConnection(transform_filter->GetOutputPort());

          auto actor = vtkSmartPointer<vtkActor>::New();
          actor->SetMapper(map);

          assembly->AddPart(actor);
        }
      }
    }
  }

  return assembly;
}

vtkSmartPointer<vtkAssembly> createMeshActors(const std::vector<pcl::PolygonMesh>& meshes)
{
  auto assembly = vtkSmartPointer<vtkAssembly>::New();
  for (const pcl::PolygonMesh& mesh : meshes)
  {
    vtkSmartPointer<vtkPolyData> mesh_poly_data = vtkSmartPointer<vtkPolyData>::New();
    pcl::VTKUtils::mesh2vtk(mesh, mesh_poly_data);

    auto map = vtkSmartPointer<vtkPolyDataMapper>::New();
    map->SetInputData(mesh_poly_data);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(map);

    assembly->AddPart(actor);
  }

  return assembly;
}

void TPPWidget::onPlan(const bool /*checked*/)
{
  try
  {
    const std::string mesh_file = ui_->line_edit_mesh->text().toStdString();
    if (mesh_file.empty())
      throw std::runtime_error("No mesh file selected");

    // Load the mesh
    pcl::PolygonMesh full_mesh;
    if (pcl::io::loadPolygonFile(mesh_file, full_mesh) < 1)
      throw std::runtime_error("Failed to load mesh from file");

    const ToolPathPlannerPipeline pipeline = pipeline_widget_->pipeline_widget->createPipeline();
    QApplication::setOverrideCursor(Qt::WaitCursor);

    // Run the mesh modifier
    std::vector<pcl::PolygonMesh> meshes = pipeline.mesh_modifier->modify(full_mesh);

    tool_paths_.clear();
    tool_paths_.reserve(meshes.size());
    std::vector<ToolPaths> unmodified_tool_paths;
    unmodified_tool_paths.reserve(meshes.size());

    for (const pcl::PolygonMesh& mesh : meshes)
    {
      // Plan the tool path
      ToolPaths path = pipeline.planner->plan(mesh);

      unmodified_tool_paths.push_back(path);
      tool_paths_.push_back(pipeline.tool_path_modifier->modify(path));
    }

    QApplication::restoreOverrideCursor();

    // Reset
    {
      renderer_->RemoveActor(mesh_fragment_actor_);
      mesh_fragment_actor_ = createMeshActors(meshes);
      renderer_->AddActor(mesh_fragment_actor_);
      mesh_fragment_actor_->SetVisibility(ui_->check_box_show_modified_mesh->isChecked());
    }

    // Render the unmodified tool paths
    {
      renderer_->RemoveActor(unmodified_tool_path_actor_);
      unmodified_tool_path_actor_ = createToolPathActors(unmodified_tool_paths, tube_filter_->GetOutputPort());
      renderer_->AddActor(unmodified_tool_path_actor_);
      unmodified_tool_path_actor_->SetVisibility(ui_->check_box_show_original_tool_path->isChecked());
    }

    // Render the modified tool paths
    {
      renderer_->RemoveActor(tool_path_actor_);
      tool_path_actor_ = createToolPathActors(tool_paths_, tube_filter_->GetOutputPort());
      renderer_->AddActor(tool_path_actor_);
      tool_path_actor_->SetVisibility(ui_->check_box_show_modified_tool_path->isChecked());
    }

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
