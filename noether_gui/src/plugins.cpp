/*! @file plugins.cpp
 *
 */

#include <noether_gui/plugin_interface.h>
// Tool path planners
//   Raster
#include <noether_gui/widgets/tool_path_planners/raster/raster_planner_widget.h>
//   Direction Generators
#include <noether_gui/widgets/tool_path_planners/raster/direction_generators/fixed_direction_generator_widget.h>
#include <noether_gui/widgets/tool_path_planners/raster/direction_generators/principal_axis_direction_generator_widget.h>
//   Origin Generators
#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/fixed_origin_generator_widget.h>
#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/aabb_origin_generator_widget.h>
#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/centroid_origin_generator_widget.h>
//   Plane Slicer Raster Planner
#include <noether_gui/widgets/tool_path_planners/raster/plane_slicer_raster_planner_widget.h>
#include <noether_gui/widgets/tool_path_planners/raster/cross_hatch_plane_slicer_raster_planner_widget.h>
//   Edge
#include <noether_gui/widgets/tool_path_planners/edge/boundary_edge_planner_widget.h>
// Tool Path Modifiers
#include <noether_gui/widgets/tool_path_modifiers/circular_lead_in_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/circular_lead_out_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/concatenate_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/direction_of_travel_orientation_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/fixed_orientation_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/moving_average_orientation_smoothing_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/raster_organization_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/snake_organization_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/standard_edge_paths_organization_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/tool_drag_orientation_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/biased_tool_drag_orientation_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/uniform_orientation_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/linear_approach_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/linear_departure_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/offset_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/uniform_spacing_spline_modifier_widget.h>
#include <noether_gui/widgets/tool_path_modifiers/uniform_spacing_linear_modifier_widget.h>
// Mesh Modifiers
#include <noether_gui/widgets/mesh_modifiers/plane_projection_modifier_widget.h>
#include <noether_gui/widgets/mesh_modifiers/euclidean_clustering_modifier_widget.h>
#include <noether_gui/widgets/mesh_modifiers/normal_estimation_pcl_widget.h>
#include <noether_gui/widgets/mesh_modifiers/normals_from_mesh_faces_modifier_widget.h>
#include <noether_gui/widgets/mesh_modifiers/fill_holes_modifier_widget.h>

#include <QWidget>
#include <QMessageBox>
#include <yaml-cpp/yaml.h>

namespace noether
{
template <typename WidgetT, typename BaseWidgetT>
struct WidgetPluginImpl : WidgetPlugin<BaseWidgetT>
{
  BaseWidgetT* create(const YAML::Node& config,
                      std::shared_ptr<const boost_plugin_loader::PluginLoader> /*loader*/,
                      QWidget* parent = nullptr) const override final
  {
    auto widget = new WidgetT(parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};

// Direction Generators
using Plugin_FixedDirectionGeneratorWidget = WidgetPluginImpl<FixedDirectionGeneratorWidget, DirectionGeneratorWidget>;

using Plugin_PrincipalAxisDirectionGeneratorWidget =
    WidgetPluginImpl<PrincipalAxisDirectionGeneratorWidget, DirectionGeneratorWidget>;

// Origin Generators
using Plugin_FixedOriginGeneratorWidget = WidgetPluginImpl<FixedOriginGeneratorWidget, OriginGeneratorWidget>;

using Plugin_CentroidOriginGeneratorWidget = WidgetPluginImpl<CentroidOriginGeneratorWidget, OriginGeneratorWidget>;

using Plugin_AABBOriginGeneratorWidget = WidgetPluginImpl<AABBOriginGeneratorWidget, OriginGeneratorWidget>;

// Tool Path Modifiers
using Plugin_StandardEdgePathsOrganizationModifierWidget =
    WidgetPluginImpl<StandardEdgePathsOrganizationModifierWidget, ToolPathModifierWidget>;

using Plugin_RasterOrganizationModifierWidget =
    WidgetPluginImpl<RasterOrganizationModifierWidget, ToolPathModifierWidget>;

using Plugin_SnakeOrganizationModifierWidget =
    WidgetPluginImpl<SnakeOrganizationModifierWidget, ToolPathModifierWidget>;

using Plugin_FixedOrientationModifierWidget = WidgetPluginImpl<FixedOrientationModifierWidget, ToolPathModifierWidget>;

using Plugin_DirectionOfTravelOrientationModifierWidget =
    WidgetPluginImpl<DirectionOfTravelOrientationModifierWidget, ToolPathModifierWidget>;

using Plugin_UniformOrientationModifierWidget =
    WidgetPluginImpl<UniformOrientationModifierWidget, ToolPathModifierWidget>;

using Plugin_MovingAverageOrientationSmoothingModifierWidget =
    WidgetPluginImpl<MovingAverageOrientationSmoothingModifierWidget, ToolPathModifierWidget>;

using Plugin_ToolDragOrientationToolPathModifierWidget =
    WidgetPluginImpl<ToolDragOrientationToolPathModifierWidget, ToolPathModifierWidget>;

using Plugin_BiasedToolDragOrientationToolPathModifierWidget =
    WidgetPluginImpl<BiasedToolDragOrientationToolPathModifierWidget, ToolPathModifierWidget>;

using Plugin_CircularLeadInToolPathModifierWidget =
    WidgetPluginImpl<CircularLeadInToolPathModifierWidget, ToolPathModifierWidget>;

using Plugin_CircularLeadOutToolPathModifierWidget =
    WidgetPluginImpl<CircularLeadOutToolPathModifierWidget, ToolPathModifierWidget>;

using Plugin_LinearApproachToolPathModifierWidget =
    WidgetPluginImpl<LinearApproachToolPathModifierWidget, ToolPathModifierWidget>;

using Plugin_LinearDepartureToolPathModifierWidget =
    WidgetPluginImpl<LinearDepartureToolPathModifierWidget, ToolPathModifierWidget>;

using Plugin_ConcatenateModifierWidget = WidgetPluginImpl<ConcatenateModifierWidget, ToolPathModifierWidget>;

using Plugin_OffsetModifierWidget = WidgetPluginImpl<OffsetModifierWidget, ToolPathModifierWidget>;

using Plugin_UniformSpacingSplineModifierWidget =
    WidgetPluginImpl<UniformSpacingSplineModifierWidget, ToolPathModifierWidget>;

using Plugin_UniformSpacingLinearModifierWidget =
    WidgetPluginImpl<UniformSpacingLinearModifierWidget, ToolPathModifierWidget>;

// Raster Tool Path Planners
struct Plugin_PlaneSlicerRasterPlannerWidget : ToolPathPlannerWidgetPlugin
{
  ToolPathPlannerWidget* create(const YAML::Node& config,
                                std::shared_ptr<const boost_plugin_loader::PluginLoader> loader,
                                QWidget* parent = nullptr) const override final
  {
    auto widget = new PlaneSlicerRasterPlannerWidget(loader, parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};

struct Plugin_CrossHatchPlaneSlicerRasterPlannerWidget : ToolPathPlannerWidgetPlugin
{
  ToolPathPlannerWidget* create(const YAML::Node& config,
                                std::shared_ptr<const boost_plugin_loader::PluginLoader> loader,
                                QWidget* parent = nullptr) const override final
  {
    auto widget = new CrossHatchPlaneSlicerRasterPlannerWidget(loader, parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};

// Edge Tool Path Planners
using Plugin_BoundaryEdgePlannerWidget = WidgetPluginImpl<BoundaryEdgePlannerWidget, ToolPathPlannerWidget>;

// Mesh Modifiers
using Plugin_PlaneProjectionMeshModifierWidget =
    WidgetPluginImpl<PlaneProjectionMeshModifierWidget, MeshModifierWidget>;
using Plugin_EuclideanClusteringMeshModifierWidget =
    WidgetPluginImpl<EuclideanClusteringMeshModifierWidget, MeshModifierWidget>;
using Plugin_NormalEstimationPCLMeshModifierWidget =
    WidgetPluginImpl<NormalEstimationPCLMeshModifierWidget, MeshModifierWidget>;
using Plugin_NormalsFromMeshFacesMeshModifierWidget =
    WidgetPluginImpl<NormalsFromMeshFacesMeshModifierWidget, MeshModifierWidget>;
using Plugin_FillHolesModifierWidget = WidgetPluginImpl<FillHolesModifierWidget, MeshModifierWidget>;

}  // namespace noether

EXPORT_DIRECTION_GENERATOR_WIDGET_PLUGIN(noether::Plugin_FixedDirectionGeneratorWidget, FixedDirection)
EXPORT_DIRECTION_GENERATOR_WIDGET_PLUGIN(noether::Plugin_PrincipalAxisDirectionGeneratorWidget, PrincipalAxis)

EXPORT_ORIGIN_GENERATOR_WIDGET_PLUGIN(noether::Plugin_FixedOriginGeneratorWidget, FixedOrigin)
EXPORT_ORIGIN_GENERATOR_WIDGET_PLUGIN(noether::Plugin_CentroidOriginGeneratorWidget, Centroid)
EXPORT_ORIGIN_GENERATOR_WIDGET_PLUGIN(noether::Plugin_AABBOriginGeneratorWidget, AABBCenter)

EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_StandardEdgePathsOrganizationModifierWidget,
                                        StandardEdgePathsOrganization)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_RasterOrganizationModifierWidget, RasterOrganization)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_SnakeOrganizationModifierWidget, SnakeOrganization)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_FixedOrientationModifierWidget, FixedOrientation)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_DirectionOfTravelOrientationModifierWidget,
                                        DirectionOfTravelOrientation)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_UniformOrientationModifierWidget, UniformOrientation)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_MovingAverageOrientationSmoothingModifierWidget,
                                        MovingAverageOrientationSmoothing)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_ToolDragOrientationToolPathModifierWidget, ToolDragOrientation)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_BiasedToolDragOrientationToolPathModifierWidget,
                                        BiasedToolDragOrientation)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_CircularLeadInToolPathModifierWidget, CircularLeadIn)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_CircularLeadOutToolPathModifierWidget, CircularLeadOut)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_LinearApproachToolPathModifierWidget, LinearApproach)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_LinearDepartureToolPathModifierWidget, LinearDeparture)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_ConcatenateModifierWidget, Concatenate)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_OffsetModifierWidget, Offset)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_UniformSpacingSplineModifierWidget, UniformSpacingSpline)
EXPORT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_UniformSpacingLinearModifierWidget, UniformSpacingLinear);

EXPORT_TPP_WIDGET_PLUGIN(noether::Plugin_PlaneSlicerRasterPlannerWidget, PlaneSlicer)
EXPORT_TPP_WIDGET_PLUGIN(noether::Plugin_CrossHatchPlaneSlicerRasterPlannerWidget, CrossHatchPlaneSlicer)
EXPORT_TPP_WIDGET_PLUGIN(noether::Plugin_BoundaryEdgePlannerWidget, Boundary)

EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_PlaneProjectionMeshModifierWidget, PlaneProjection)
EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_EuclideanClusteringMeshModifierWidget, EuclideanClustering)
EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_NormalEstimationPCLMeshModifierWidget, NormalEstimationPCL)
EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_NormalsFromMeshFacesMeshModifierWidget, NormalsFromMeshFaces)
EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(noether::Plugin_FillHolesModifierWidget, FillHoles)
