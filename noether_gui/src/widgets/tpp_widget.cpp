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
#include <pcl/io/ply_io.h>

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


#include <vtkProp3DCollection.h>

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
  , mesh_fragment_mapper_(vtkOpenGLPolyDataMapper::New())
  , mesh_fragment_actor_(vtkOpenGLActor::New())
  , combined_mesh_fragments_(vtkSmartPointer<vtkPolyData>::New())
  , tool_path_actor_(vtkAssembly::New())
  , unmodified_tool_path_actor_(vtkAssembly::New())
  , axes_(vtkAxes::New())
  , tube_filter_(vtkTubeFilter::New())
{
  ui_->setupUi(this);

  ui_->scroll_area->setWidget(pipeline_widget_);
  ui_->splitter->addWidget(render_widget_);

  // Set up the VTK objects
  renderer_->SetBackground(0.2, 0.2, 0.2);

  // Original mesh mapper/actor
  mesh_actor_->SetMapper(mesh_mapper_);
  renderer_->AddActor(mesh_actor_);

  // Mesh fragment mapper/actor
  mesh_fragment_mapper_->SetInputData(combined_mesh_fragments_);
  mesh_fragment_actor_->SetMapper(mesh_fragment_mapper_);
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
  connect(ui_->push_button_load_configuration, &QPushButton::clicked, this, &TPPWidget::onLoadConfiguration);
  connect(ui_->push_button_save_configuration, &QPushButton::clicked, this, &TPPWidget::onSaveConfiguration);
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

TPPWidget::~TPPWidget()
{
  renderer_->Delete();
  mesh_mapper_->Delete();
  mesh_actor_->Delete();
  mesh_fragment_mapper_->Delete();
  mesh_fragment_actor_->Delete();
  tool_path_actor_->Delete();
  unmodified_tool_path_actor_->Delete();
  axes_->Delete();
  tube_filter_->Delete();
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

vtkAssembly* createToolPathActors(const std::vector<ToolPaths>& tool_paths,
                                  vtkAlgorithmOutput* waypoint_shape_output_port)
{
  auto assembly = vtkAssembly::New();
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

// Function to combine two pcl::PolygonMesh instances
pcl::PolygonMesh combineMeshes(const pcl::PolygonMesh& mesh1, const pcl::PolygonMesh& mesh2) {
    pcl::PolygonMesh combinedMesh;

    // Combine vertices
    pcl::PointCloud<pcl::PointXYZ> combinedCloud;
    pcl::fromPCLPointCloud2(mesh1.cloud, combinedCloud);
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    pcl::fromPCLPointCloud2(mesh2.cloud, cloud2);

    combinedCloud += cloud2;

    pcl::toPCLPointCloud2(combinedCloud, combinedMesh.cloud);

    // Combine faces
    // Add faces for mesh 1
    combinedMesh.polygons.insert(
        combinedMesh.polygons.end(), mesh1.polygons.begin(), mesh1.polygons.end());

    // Offset the indices of the second mesh's faces to account for the combined vertices
    size_t offset = mesh1.cloud.width * mesh1.cloud.height;
    for (auto& polygon : mesh2.polygons) {
        pcl::Vertices newPolygon;
        for (auto index : polygon.vertices) {
            newPolygon.vertices.push_back(index + offset);
        }
        combinedMesh.polygons.push_back(newPolygon);
    }

    return combinedMesh;
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

    const ToolPathPlannerPipeline pipeline = pipeline_widget_->createPipeline();
    QApplication::setOverrideCursor(Qt::WaitCursor);

    // Run the mesh modifier
    std::vector<pcl::PolygonMesh> meshes = pipeline.mesh_modifier->modify(full_mesh);

    tool_paths_.clear();
    tool_paths_.reserve(meshes.size());

    pcl::PolygonMesh combined_mesh_fragments;
    std::vector<ToolPaths> unmodified_tool_paths;
    std::string file_name = "/tmp/comb_frag_mesh_test";
    for(std::size_t i = 0; i < meshes.size(); ++i)
    {
      const pcl::PolygonMesh& mesh = meshes[i];
      if (i == 0)
      {
        combined_mesh_fragments = mesh;
      }

      else
      {
        combined_mesh_fragments = combineMeshes(combined_mesh_fragments, mesh);
      }

      pcl::io::savePLYFile(file_name + std::to_string(i) + ".ply", combined_mesh_fragments);

      // Plan the tool path
      ToolPaths path = pipeline.planner->plan(mesh);

      unmodified_tool_paths.push_back(path);
      tool_paths_.push_back(pipeline.tool_path_modifier->modify(path));
    }

    QApplication::restoreOverrideCursor();

    // Reset
    {
      pcl::VTKUtils::mesh2vtk(combined_mesh_fragments, combined_mesh_fragments_);
      mesh_fragment_mapper_->Update();
      mesh_fragment_mapper_->SetInputData(combined_mesh_fragments_);

    }

    // Render the unmodified tool paths
    {
      renderer_->RemoveActor(unmodified_tool_path_actor_);
      unmodified_tool_path_actor_->Delete();
      unmodified_tool_path_actor_ = createToolPathActors(unmodified_tool_paths, tube_filter_->GetOutputPort());
      renderer_->AddActor(unmodified_tool_path_actor_);
      unmodified_tool_path_actor_->SetVisibility(ui_->check_box_show_original_tool_path->isChecked());
    }

    // Render the modified tool paths
    {
      renderer_->RemoveActor(tool_path_actor_);
      tool_path_actor_->Delete();
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
