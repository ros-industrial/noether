/*! @file plugins.cpp
 *
 */

#include <noether_gui/plugin_interface.h>
// Direction Generators
#include <noether_gui/widgets/tool_path_planners/raster/direction_generators/fixed_direction_generator_widget.h>
#include <noether_gui/widgets/tool_path_planners/raster/direction_generators/principal_axis_direction_generator_widget.h>
// Origin Generators
#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/fixed_origin_generator_widget.h>
#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/aabb_center_origin_generator_widget.h>
#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/centroid_origin_generator_widget.h>
// Tool path planners
#include <noether_gui/widgets/tool_path_planners/raster/raster_planner_widget.h>
#include <noether_gui/widgets/tool_path_planners/raster/cross_hatch_plane_slicer_raster_planner_widget.h>
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
#include <noether_gui/widgets/mesh_modifiers/euclidean_clustering_modifier_widget.h>
#include <noether_gui/widgets/mesh_modifiers/fill_holes_modifier_widget.h>
#include <noether_gui/widgets/mesh_modifiers/normal_estimation_pcl_widget.h>
#include <noether_gui/widgets/mesh_modifiers/normals_from_mesh_faces_modifier_widget.h>
#include <noether_gui/widgets/mesh_modifiers/ransac_cylinder_fit_modifier_widget.h>
#include <noether_gui/widgets/mesh_modifiers/ransac_plane_fit_modifier_widget.h>
#include <noether_gui/widgets/mesh_modifiers/upsampling_modifier_widget.h>

#include <QWidget>
#include <QMessageBox>
#include <yaml-cpp/yaml.h>

namespace noether
{
// Mesh Modifiers
EXPORT_SIMPLE_MESH_MODIFIER_WIDGET_PLUGIN(EuclideanClusteringMeshModifierWidget, EuclideanClustering)
EXPORT_SIMPLE_MESH_MODIFIER_WIDGET_PLUGIN(FillHolesModifierWidget, FillHoles)
EXPORT_SIMPLE_MESH_MODIFIER_WIDGET_PLUGIN(NormalEstimationPCLMeshModifierWidget, NormalEstimationPCL)
EXPORT_SIMPLE_MESH_MODIFIER_WIDGET_PLUGIN(NormalsFromMeshFacesMeshModifierWidget, NormalsFromMeshFaces)
EXPORT_SIMPLE_MESH_MODIFIER_WIDGET_PLUGIN(RansacCylinderFitMeshModifierWidget, RansacCylinderFit)
EXPORT_SIMPLE_MESH_MODIFIER_WIDGET_PLUGIN(RansacCylinderProjectionMeshModifierWidget, RansacCylinderProjection)
//! [GUI Plugin Alias Correspondence]
EXPORT_SIMPLE_MESH_MODIFIER_WIDGET_PLUGIN(RansacPlaneProjectionMeshModifierWidget, RansacPlaneProjection)
//! [GUI Plugin Alias Correspondence]
EXPORT_SIMPLE_MESH_MODIFIER_WIDGET_PLUGIN(UpsamplingMeshModifierWidget, Upsampling);

// Direction Generators
EXPORT_SIMPLE_DIRECTION_GENERATOR_WIDGET_PLUGIN(FixedDirectionGeneratorWidget, FixedDirection)
EXPORT_SIMPLE_DIRECTION_GENERATOR_WIDGET_PLUGIN(PrincipalAxisDirectionGeneratorWidget, PrincipalAxis)

// Origin Generators
EXPORT_SIMPLE_ORIGIN_GENERATOR_WIDGET_PLUGIN(FixedOriginGeneratorWidget, FixedOrigin)
EXPORT_SIMPLE_ORIGIN_GENERATOR_WIDGET_PLUGIN(CentroidOriginGeneratorWidget, Centroid)
EXPORT_SIMPLE_ORIGIN_GENERATOR_WIDGET_PLUGIN(AABBCenterOriginGeneratorWidget, AABBCenter)

// Tool Path Planners
struct Plugin_CrossHatchPlaneSlicerRasterPlannerWidget : WidgetPlugin
{
  BaseWidget* create(const YAML::Node& config,
                     std::shared_ptr<const WidgetFactory> factory,
                     QWidget* parent = nullptr) const override final
  {
    auto widget = new CrossHatchPlaneSlicerRasterPlannerWidget(factory, parent);

    // Attempt to configure the widget
    if (!config.IsNull())
    {
      try
      {
        widget->configure(config);
      }
      catch (const std::exception&)
      {
        // Delete the widget to prevent it from showing up in the GUI outside of the appropriate layout
        widget->deleteLater();
        throw;
      }
    }

    return widget;
  }
};
EXPORT_TOOL_PATH_PLANNER_WIDGET_PLUGIN(noether::Plugin_CrossHatchPlaneSlicerRasterPlannerWidget, CrossHatchPlaneSlicer)

// Tool Path Modifiers
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(StandardEdgePathsOrganizationModifierWidget,
                                               StandardEdgePathsOrganization)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(RasterOrganizationModifierWidget, RasterOrganization)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(SnakeOrganizationModifierWidget, SnakeOrganization)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(FixedOrientationModifierWidget, FixedOrientation)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(DirectionOfTravelOrientationModifierWidget, DirectionOfTravelOrientation)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(UniformOrientationModifierWidget, UniformOrientation)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(MovingAverageOrientationSmoothingModifierWidget,
                                               MovingAverageOrientationSmoothing)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(ToolDragOrientationToolPathModifierWidget, ToolDragOrientation)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(BiasedToolDragOrientationToolPathModifierWidget,
                                               BiasedToolDragOrientation)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(CircularLeadInToolPathModifierWidget, CircularLeadIn)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(CircularLeadOutToolPathModifierWidget, CircularLeadOut)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(LinearApproachToolPathModifierWidget, LinearApproach)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(LinearDepartureToolPathModifierWidget, LinearDeparture)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(ConcatenateModifierWidget, Concatenate)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(OffsetModifierWidget, Offset)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(UniformSpacingSplineModifierWidget, UniformSpacingSpline)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_WIDGET_PLUGIN(UniformSpacingLinearModifierWidget, UniformSpacingLinear)

}  // namespace noether

//! [GUI Plugins Example Simple]
// Include the plugin interface header
#include <noether_gui/plugin_interface.h>

// Include the widget header
#include <noether_gui/widgets/tool_path_planners/edge/boundary_edge_planner_widget.h>

namespace noether
{
// For tool path planning component widgets that can be fully configured by YAML serialization, invoke the
// `EXPORT_SIMPLE_<COMPONENT>_WIDGET_PLUGIN` macro. This macro instantiates the SimpleWidgetPlugin template for the
// input class name (first argument) and exports that plugin with the input alias (second argument).
// Ideally, the alias matches the alias of the corresponding tool path planning component plugin.
EXPORT_SIMPLE_TOOL_PATH_PLANNER_WIDGET_PLUGIN(BoundaryEdgePlannerWidget, Boundary)

}  // namespace noether
//! [GUI Plugins Example Simple]

//! [GUI Plugins Example Complex]
// Include the plugin interface header
#include <noether_gui/plugin_interface.h>

// Include the widget header
#include <noether_gui/widgets/tool_path_planners/raster/plane_slicer_raster_planner_widget.h>

namespace noether
{
// For a complex plugin that cannot be configured through YAML serialization alone, implement a custom plugin class.
struct Plugin_PlaneSlicerRasterPlannerWidget : WidgetPlugin
{
  BaseWidget* create(const YAML::Node& config,
                     std::shared_ptr<const WidgetFactory> factory,
                     QWidget* parent = nullptr) const override final
  {
    auto widget = new PlaneSlicerRasterPlannerWidget(factory, parent);

    // Attempt to configure the widget
    if (!config.IsNull())
    {
      try
      {
        widget->configure(config);
      }
      catch (const std::exception&)
      {
        // Delete the widget to prevent it from showing up in the GUI outside of the appropriate layout
        widget->deleteLater();
        throw;
      }
    }

    return widget;
  }
};

// Export the plugin class with an alias using the `EXPORT_<COMPONENT>_WIDGET_PLUGIN` macro
// Ideally, the alias matches the alias of the corresponding tool path planning component plugin
EXPORT_TOOL_PATH_PLANNER_WIDGET_PLUGIN(Plugin_PlaneSlicerRasterPlannerWidget, PlaneSlicer)

}  // namespace noether
//! [GUI Plugins Example Complex]
