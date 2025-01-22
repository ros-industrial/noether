#include <noether_gui/widgets/tpp_widget.h>
#include "ui_tpp_widget.h"
#include <noether_gui/widgets/configurable_tpp_pipeline_widget.h>
#include <noether_gui/widgets/tpp_pipeline_widget.h>
#include <noether_gui/utils.h>

#include <pcl/io/vtk_lib_io.h>
#include <QColorDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>
#include <QTextStream>
#include <yaml-cpp/yaml.h>

// Rendering includes
#ifndef VTK_MAJOR_VERSION
#include <vtkVersionMacros.h>
#endif
#if VTK_MAJOR_VERSION > 7
#include <QVTKOpenGLNativeWidget.h>
#else
#include <QVTKWidget.h>
#endif
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
#include <vtkProperty.h>
#include <vtkColorSeries.h>
#include <vtkLine.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

namespace noether
{
TPPWidget::TPPWidget(boost_plugin_loader::PluginLoader loader, QWidget* parent)
  : QMainWindow(parent)
  , ui_(new Ui::TPP())
  , pipeline_widget_(new ConfigurableTPPPipelineWidget(std::move(loader), this))
  , render_widget_(new RenderWidget(this))
  , renderer_(vtkSmartPointer<vtkOpenGLRenderer>::New())
  , mesh_mapper_(vtkSmartPointer<vtkOpenGLPolyDataMapper>::New())
  , mesh_actor_(vtkSmartPointer<vtkOpenGLActor>::New())
  , mesh_fragment_actor_(vtkSmartPointer<vtkAssembly>::New())
  , tool_path_actor_(vtkSmartPointer<vtkAssembly>::New())
  , connected_path_actor_(vtkSmartPointer<vtkAssembly>::New())
  , unmodified_tool_path_actor_(vtkSmartPointer<vtkAssembly>::New())
  , unmodified_connected_path_actor_(vtkSmartPointer<vtkAssembly>::New())
  , axes_(vtkSmartPointer<vtkAxes>::New())
  , tube_filter_(vtkSmartPointer<vtkTubeFilter>::New())
{
  ui_->setupUi(this);

  overwriteWidget(ui_->verticalLayout, ui_->widget, pipeline_widget_);
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
  mesh_actor_->SetVisibility(ui_->action_show_unmodified_mesh->isChecked());
  mesh_fragment_actor_->SetVisibility(ui_->action_show_modified_mesh->isChecked());
  unmodified_tool_path_actor_->SetVisibility(ui_->action_show_unmodified_tool_path->isChecked());
  tool_path_actor_->SetVisibility(ui_->action_show_modified_tool_path->isChecked());
  unmodified_connected_path_actor_->SetVisibility(ui_->action_show_unmodified_tool_path_lines->isChecked());
  connected_path_actor_->SetVisibility(ui_->action_show_modified_tool_path_lines->isChecked());

  // Connect signals
  connect(ui_->action_load_mesh, &QAction::triggered, this, &TPPWidget::onLoadMesh);
  connect(ui_->action_execute_pipeline, &QAction::triggered, this, &TPPWidget::onPlan);

  connect(ui_->action_show_unmodified_mesh, &QAction::triggered, this, &TPPWidget::onShowOriginalMesh);
  connect(ui_->action_show_modified_mesh, &QAction::triggered, this, &TPPWidget::onShowModifiedMesh);
  connect(ui_->action_show_unmodified_tool_path, &QAction::triggered, this, &TPPWidget::onShowUnmodifiedToolPath);
  connect(ui_->action_show_modified_tool_path, &QAction::triggered, this, &TPPWidget::onShowModifiedToolPath);
  connect(ui_->action_show_unmodified_tool_path_lines,
          &QAction::triggered,
          this,
          &TPPWidget::onShowUnmodifiedConnectedPath);
  connect(
      ui_->action_show_modified_tool_path_lines, &QAction::triggered, this, &TPPWidget::onShowModifiedConnectedPath);

  connect(ui_->action_load_config,
          &QAction::triggered,
          pipeline_widget_,
          &ConfigurableTPPPipelineWidget::onLoadConfiguration);
  connect(ui_->action_save_config,
          &QAction::triggered,
          pipeline_widget_,
          &ConfigurableTPPPipelineWidget::onSaveConfiguration);

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

void TPPWidget::onShowUnmodifiedConnectedPath(const bool checked)
{
  unmodified_connected_path_actor_->SetVisibility(checked);
  render_widget_->GetRenderWindow()->Render();
  render_widget_->GetRenderWindow()->Render();
}

void TPPWidget::onShowUnmodifiedToolPath(const bool checked)
{
  unmodified_tool_path_actor_->SetVisibility(checked);
  render_widget_->GetRenderWindow()->Render();
  render_widget_->GetRenderWindow()->Render();
}

void TPPWidget::onShowModifiedConnectedPath(const bool checked)
{
  connected_path_actor_->SetVisibility(checked);
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
  // Render the mesh
  vtkSmartPointer<vtkAbstractPolyDataReader> reader;
  if (file.endsWith(".ply"))
    reader = vtkSmartPointer<vtkPLYReader>::New();
  else if (file.endsWith(".stl"))
    reader = vtkSmartPointer<vtkSTLReader>::New();
  else
    return;

  mesh_file_ = file.toStdString();

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
  const QString file = QFileDialog::getOpenFileName(this, "Load mesh file", "", "Mesh files (*.ply *.stl)");
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

vtkSmartPointer<vtkActor> createLineActor(const Eigen::Isometry3d& point1,
                                          const Eigen::Isometry3d& point2,
                                          LineStyle lineStyle)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(point1.translation().data());
  points->InsertNextPoint(point2.translation().data());

  vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
  line->GetPointIds()->SetId(0, 0);  // the second 0 is the index of the point in vtkPoints (point1)
  line->GetPointIds()->SetId(1, 1);  // the second 1 is the index of the point in vtkPoints (point2)

  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
  lines->InsertNextCell(line);

  vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
  linesPolyData->SetPoints(points);
  linesPolyData->SetLines(lines);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(linesPolyData);

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  vtkSmartPointer<vtkProperty> property = vtkSmartPointer<vtkProperty>::New();

  switch (lineStyle)
  {
    case LineStyle::INTRA_SEGMENT:
      property->SetLineWidth(3.0);
      property->SetColor(0.27, 0.74, 0.2);
      break;
    case LineStyle::INTER_SEGMENT:
      property->SetLineWidth(1.0);
      property->SetColor(1, 0.8, 0);
      break;
    case LineStyle::INTER_PATH:
      property->SetLineWidth(0.5);
      property->SetColor(1, 0.271, 0.043);
      break;
  }

  actor->SetProperty(property);

  return actor;
}

/*
 *  Create a polyline between all points of the tool path
 *
 *  @param tool_paths The tool paths to create the polyline from
 *  @param waypoint_shape_output_port The output port of the waypoint shape
 *  @return The assembly containing the polyline
 */
vtkSmartPointer<vtkAssembly> createToolPathPolylineActor(const std::vector<ToolPaths>& tool_paths,
                                                         vtkAlgorithmOutput* waypoint_shape_output_port)
{
  auto assembly = vtkSmartPointer<vtkAssembly>::New();

  for (const ToolPaths& fragment : tool_paths)
  {
    for (int i = 0; i < fragment.size(); i++)
    {
      const ToolPath& tool_path = fragment[i];

      for (int j = 0; j < tool_path.size(); j++)
      {
        const ToolPathSegment& segment = tool_path[j];

        for (int k = 0; k < segment.size() - 1; k++)
        {
          const Eigen::Isometry3d& point = segment[k];
          const Eigen::Isometry3d& next_point = segment[k + 1];

          auto actor = createLineActor(point, next_point, LineStyle::INTRA_SEGMENT);
          assembly->AddPart(actor);
        }

        if (j < tool_path.size() - 1)
        {
          const ToolPathSegment& next_segment = tool_path[j + 1];
          const Eigen::Isometry3d& w1 = segment.back();
          const Eigen::Isometry3d& w2 = next_segment.front();

          auto actor = createLineActor(w1, w2, LineStyle::INTER_SEGMENT);
          assembly->AddPart(actor);
        }
      }

      if (i < fragment.size() - 1)
      {
        const ToolPath& next_tool_path = fragment[i + 1];
        const Eigen::Isometry3d& w1 = tool_path.back().back();
        const Eigen::Isometry3d& w2 = next_tool_path.front().front();

        auto actor = createLineActor(w1, w2, LineStyle::INTER_PATH);
        assembly->AddPart(actor);
      }
    }
  }

  return assembly;
}

vtkSmartPointer<vtkAssembly> createMeshActors(const std::vector<pcl::PolygonMesh>& meshes)
{
  auto assembly = vtkSmartPointer<vtkAssembly>::New();

  // Create a color series to differentiate the meshes
  auto color_series = vtkSmartPointer<vtkColorSeries>::New();
  color_series->SetColorScheme(vtkColorSeries::ColorSchemes::BREWER_QUALITATIVE_SET1);
  color_series->SetNumberOfColors(meshes.size());

  for (std::size_t i = 0; i < meshes.size(); ++i)
  {
    vtkSmartPointer<vtkPolyData> mesh_poly_data = vtkSmartPointer<vtkPolyData>::New();
    pcl::VTKUtils::mesh2vtk(meshes[i], mesh_poly_data);

    auto map = vtkSmartPointer<vtkPolyDataMapper>::New();
    map->SetInputData(mesh_poly_data);
    map->SetScalarVisibility(false);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(map);

    // Set the color of the actor from the color series
    vtkTuple<double, 3> color = color_series->GetColorRepeating(i).Cast<double>();
    double r = color[0] / 255.0;
    double g = color[1] / 255.0;
    double b = color[2] / 255.0;
    actor->GetProperty()->SetColor(r, g, b);

    assembly->AddPart(actor);
  }

  return assembly;
}

void TPPWidget::onPlan(const bool /*checked*/)
{
  try
  {
    if (mesh_file_.empty())
      throw std::runtime_error("No mesh file selected");

    // Load the mesh
    pcl::PolygonMesh full_mesh;
    if (pcl::io::loadPolygonFile(mesh_file_, full_mesh) < 1)
      throw std::runtime_error("Failed to load mesh from file");

    const ToolPathPlannerPipeline pipeline = pipeline_widget_->createPipeline();
    QApplication::setOverrideCursor(Qt::WaitCursor);

    // Run the mesh modifier
    std::vector<pcl::PolygonMesh> meshes;
    try
    {
      meshes = pipeline.mesh_modifier->modify(full_mesh);
    }
    catch (const std::exception& ex)
    {
      std::throw_with_nested(std::runtime_error("Error invoking mesh modifier"));
    }

    tool_paths_.clear();
    tool_paths_.reserve(meshes.size());
    std::vector<ToolPaths> unmodified_tool_paths;
    unmodified_tool_paths.reserve(meshes.size());

    for (std::size_t i = 0; i < meshes.size(); ++i)
    {
      const pcl::PolygonMesh& mesh = meshes[i];

      // Plan the tool path
      ToolPaths path;
      try
      {
        path = pipeline.planner->plan(mesh);
      }
      catch (const std::exception& ex)
      {
        std::stringstream ss;
        ss << "Error invoking tool path planner on mesh at index " << i << " in the TPP pipeline.";
        std::throw_with_nested(std::runtime_error(ss.str()));
      }

      unmodified_tool_paths.push_back(path);

      try
      {
        tool_paths_.push_back(pipeline.tool_path_modifier->modify(path));
      }
      catch (const std::exception& ex)
      {
        std::stringstream ss;
        ss << "Error invoking tool path modifier for tool path at index " << i << " in the TPP pipeline.";
        std::throw_with_nested(std::runtime_error(ss.str()));
      }
    }

    QApplication::restoreOverrideCursor();

    // Reset
    {
      renderer_->RemoveActor(mesh_fragment_actor_);
      mesh_fragment_actor_ = createMeshActors(meshes);
      renderer_->AddActor(mesh_fragment_actor_);
      mesh_fragment_actor_->SetVisibility(ui_->action_show_modified_mesh->isChecked());
    }

    // Render the unmodified tool paths
    {
      renderer_->RemoveActor(unmodified_tool_path_actor_);
      unmodified_tool_path_actor_ = createToolPathActors(unmodified_tool_paths, tube_filter_->GetOutputPort());
      renderer_->AddActor(unmodified_tool_path_actor_);
      unmodified_tool_path_actor_->SetVisibility(ui_->action_show_unmodified_tool_path->isChecked());
    }

    // Render the unmodified connected paths
    {
      renderer_->RemoveActor(unmodified_connected_path_actor_);
      unmodified_connected_path_actor_ =
          createToolPathPolylineActor(unmodified_tool_paths, tube_filter_->GetOutputPort());
      renderer_->AddActor(unmodified_connected_path_actor_);
      unmodified_connected_path_actor_->SetVisibility(ui_->action_show_unmodified_tool_path_lines->isChecked());
    }

    // Render the modified tool paths
    {
      renderer_->RemoveActor(tool_path_actor_);
      tool_path_actor_ = createToolPathActors(tool_paths_, tube_filter_->GetOutputPort());
      renderer_->AddActor(tool_path_actor_);
      tool_path_actor_->SetVisibility(ui_->action_show_modified_tool_path->isChecked());
    }

    // Render the modified connected paths
    {
      renderer_->RemoveActor(connected_path_actor_);
      connected_path_actor_ = createToolPathPolylineActor(tool_paths_, tube_filter_->GetOutputPort());
      renderer_->AddActor(connected_path_actor_);
      connected_path_actor_->SetVisibility(ui_->action_show_modified_tool_path_lines->isChecked());
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
