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
// Direction Generators
EXPORT_DEFAULT_DIRECTION_GENERATOR_WIDGET_PLUGIN(FixedDirectionGeneratorWidget, FixedDirection)
EXPORT_DEFAULT_DIRECTION_GENERATOR_WIDGET_PLUGIN(PrincipalAxisDirectionGeneratorWidget, PrincipalAxis)

// Origin Generators
EXPORT_DEFAULT_ORIGIN_GENERATOR_WIDGET_PLUGIN(FixedOriginGeneratorWidget, FixedOrigin)
EXPORT_DEFAULT_ORIGIN_GENERATOR_WIDGET_PLUGIN(CentroidOriginGeneratorWidget, Centroid)
EXPORT_DEFAULT_ORIGIN_GENERATOR_WIDGET_PLUGIN(AABBOriginGeneratorWidget, AABBCenter)

// Tool Path Modifiers
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(StandardEdgePathsOrganizationModifierWidget,
                                                StandardEdgePathsOrganization)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(RasterOrganizationModifierWidget, RasterOrganization)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(SnakeOrganizationModifierWidget, SnakeOrganization)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(FixedOrientationModifierWidget, FixedOrientation)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(DirectionOfTravelOrientationModifierWidget,
                                                DirectionOfTravelOrientation)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(UniformOrientationModifierWidget, UniformOrientation)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(MovingAverageOrientationSmoothingModifierWidget,
                                                MovingAverageOrientationSmoothing)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(ToolDragOrientationToolPathModifierWidget, ToolDragOrientation)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(BiasedToolDragOrientationToolPathModifierWidget,
                                                BiasedToolDragOrientation)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(CircularLeadInToolPathModifierWidget, CircularLeadIn)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(CircularLeadOutToolPathModifierWidget, CircularLeadOut)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(LinearApproachToolPathModifierWidget, LinearApproach)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(LinearDepartureToolPathModifierWidget, LinearDeparture)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(ConcatenateModifierWidget, Concatenate)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(OffsetModifierWidget, Offset)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(UniformSpacingSplineModifierWidget, UniformSpacingSpline)
EXPORT_DEFAULT_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(UniformSpacingLinearModifierWidget, UniformSpacingLinear)

// Raster Tool Path Planners
struct Plugin_PlaneSlicerRasterPlannerWidget : WidgetPlugin
{
  BaseWidget* create(const YAML::Node& config,
                     std::shared_ptr<const GuiFactory> factory,
                     QWidget* parent = nullptr) const override final
  {
    auto widget = new PlaneSlicerRasterPlannerWidget(factory, parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};
EXPORT_TOOL_PATH_PLANNER_WIDGET_PLUGIN(noether::Plugin_PlaneSlicerRasterPlannerWidget, PlaneSlicer)

struct Plugin_CrossHatchPlaneSlicerRasterPlannerWidget : WidgetPlugin
{
  BaseWidget* create(const YAML::Node& config,
                     std::shared_ptr<const GuiFactory> factory,
                     QWidget* parent = nullptr) const override final
  {
    auto widget = new CrossHatchPlaneSlicerRasterPlannerWidget(factory, parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};
EXPORT_TOOL_PATH_PLANNER_WIDGET_PLUGIN(noether::Plugin_CrossHatchPlaneSlicerRasterPlannerWidget, CrossHatchPlaneSlicer)

// Edge Tool Path Planners
EXPORT_DEFAULT_TOOL_PATH_PLANNER_WIDGET_PLUGIN(BoundaryEdgePlannerWidget, Boundary)

// Mesh Modifiers
EXPORT_DEFAULT_MESH_MODIFIER_WIDGET_PLUGIN(PlaneProjectionMeshModifierWidget, PlaneProjection)
EXPORT_DEFAULT_MESH_MODIFIER_WIDGET_PLUGIN(EuclideanClusteringMeshModifierWidget, EuclideanClustering)
EXPORT_DEFAULT_MESH_MODIFIER_WIDGET_PLUGIN(NormalEstimationPCLMeshModifierWidget, NormalEstimationPCL)
EXPORT_DEFAULT_MESH_MODIFIER_WIDGET_PLUGIN(NormalsFromMeshFacesMeshModifierWidget, NormalsFromMeshFaces)
EXPORT_DEFAULT_MESH_MODIFIER_WIDGET_PLUGIN(FillHolesModifierWidget, FillHoles)

}  // namespace noether
