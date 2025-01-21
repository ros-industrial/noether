namespace noether
{

// Main page
/**
@mainpage

<a href="https://github.com/ros-industrial/noether">Noether</a> provides software for performing tool path planning on
arbitrary mesh surfaces. See the following pages for more information on using Noether.

<ul>
  <li>@subpage page_concepts</li>
  <li>@subpage page_getting_started</li>
  <li>@subpage page_customization</li>
  <li>@subpage page_additional_resources</li>
</ul>

*/

// Concepts page
/**
@page page_concepts Concepts

@tableofcontents

We define tool path planning as the generation of an ordered set of waypoints for a robotic process (e.g., painting,
sanding, inspection, routing, etc.) based an input mesh geometry. The idea is that the tool of the robot should traverse
through this set of waypoints in order to accomplish the desired task. Definitions of the concepts of waypoints and tool
paths can be found in @ref types.

@section s_pipeline Tool Path Planning Pipeline

We think of the tool path planning process as a pipeline consisting of 3 steps:

<ol>
  <li><b>Mesh modification</b></li>
  <li><b>Tool path planning</b></li>
  <li><b>Tool path modification</b></li>
</ol>

@subsection ss_mesh_modifier Mesh Modification

Often input geometry (specifically meshes) does not actually reflect the surface from which we want to generate tool
paths. Usually we want to process the mesh in various ways, such as smoothing, segmentation, or primitive fitting and
projection. A MeshModifier receives a single input mesh, applies these modification operations, and returns at least one
output mesh.

@subsection ss_tool_path_planner Tool Path Planning

This is the generation of waypoints directly from the modified mesh(es).
Planners generally create waypoints directly on the surface of the input modified mesh(es).
A ToolPathPlanner receives a mesh and returns a set of ToolPath objects.

@subsection ss_tool_path_modifier Tool Path Modification

Many robot processes require application specific changes or additions to a tool path.
These include operations like order reorganization, pose offsets, approaches/departures, etc.
Instead of building these changes directly into customized tool path planners, we implement them as a separate step in
the pipeline. A ToolPathModifier takes in the output type of a ToolPathPlanner (i.e. ToolPaths) and returns a modified
ToolPaths object with these process-specific changes.

@subsubsection sss_one_time_modifier One-time Tool Path Modifiers

There should be a strong preference for constructing modifiers that have no additional effect when run repeatedly.
Since some modifications (such as adding an extra raster) will not meet this requirement, it will only apply to a
specialized subclass: OneTimeToolPathModifier.

As an example: While a ToolPathModifier could be implemented via a function that reverses the direction of odd-indexed
rasters, running that function twice would undo the desired changes. Instead, a more desirable OneTimeToolPathModifier
might set the direction of every even-indexed raster to the direction of the zero-indexed raster, and set the direction
of every odd-indexed raster to the opposite. This would result in no additional change if run multiple times.

@image html docs/modifier.png

*/

// Getting started page
/**
@page page_getting_started Getting Started

@section Build

Nominally, this project is ROS-independent, but it is convenient to use the ROS dependency management and build tools to
build the repository.

Clone the repository into a workspace, download the dependencies, and build the workspace.

    cd <ws>
    vcs import src < dependencies.repos
    rosdep install --from-paths src -iry
    <colcon/catkin> build

@section s_gui GUI

Noether provides a GUI for configuring and operating a @ref s_pipeline.

@image html docs/gui.png width=90%

Run the application using the following command:

    ros2 run noether_gui noether_gui_app

@note
If you encounter the error `error while loading shared libraries: libjawt.so: cannot open shared object file: No such
file or directory`, try manually adding the location of the unlinked Java libraries:

    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/jvm/java-11-openjdk-amd64/lib
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/jvm/java-11-openjdk-amd64/lib/server

*/

// Customization Page
/**
@page page_customization Customization

@tableofcontents

Custom behavior can be added by providing a class that overrides one of the @ref interfaces.
See the implementations in @ref mesh_modifiers, @ref tool_path_planners, and @ref tool_path_modifiers for reference.

@section s_gui_plugins GUI Plugins

@par
The Noether @ref s_gui is a framework that allows user-developed plugins to provide UI elements for configuring the @ref
s_pipeline. The GUI has hooks for displaying widgets for all available MeshModifier, ToolPathPlanner, and
ToolPathModifier implementations.

@par
At run-time, the GUI searches for plugins that can provide the widgets to configure these tool path planning concept
@ref interfaces. The GUI automatically finds plugins defined in this repository. User-developed plugins can also be
found by the GUI by adding the names of the plugin libraries to the `NOETHER_PLUGIN_LIBS` environment variable.

@par
This plugin-based GUI architecture allows end-users to leverage the out-of-the-box GUI infrastructure and plugins that
come with Noether, while still enabling private development of custom capabilities or customizations to the front-end of
existing Noether classes. Read the following sections to learn how to implement your own plugin for the Noether GUI.

@section s_gui_how_to How to create your own Noether GUI plugin

*/

// Additional resources page
/**
@page page_additional_resources Additional Resources

@section s_noether_roscon_2024 ROSCon 2024 Workshop

See this <a href="https://github.com/marip8/noether_roscon_2024">repository</a> for a workshop that was put on at <a
href="https://roscon.ros.org/2024/">ROSCon 2024</a>. The repository includes exercises and demonstrations for creating
custom implementations of Noether interfaces, including integration with the GUI.

*/

// Define groups
/**
  @defgroup types Types
  @defgroup interfaces Interfaces
  @defgroup mesh_modifiers Mesh Modifiers
  @defgroup tool_path_planners Tool Path Planners
  @{
    @defgroup raster_planners Raster Planners
    @{
      @defgroup direction_generators Direction Generators
      @defgroup origin_generators Origin Generators
    @}
    @defgroup edge_planners Edge Planners
  @}
  @defgroup tool_path_modifiers Tool Path Modifiers
  @defgroup gui_interfaces GUI Interfaces
  @{
    @defgroup gui_interfaces_widgets Widgets
    @defgroup gui_interfaces_plugins Widget Plugins
  @}
*/

// Documentation for types
/**
@addtogroup types

@brief Common data types used for tool path planning

*/

// Documentation for interfaces
/**
@addtogroup interfaces

@brief Abstract interfaces for the various concepts of tool path planning.

@details The image below shows the architecture for the tool path planning components.

@image html static/architecture.png

*/

// Documentation for mesh modifiers
/**
@addtogroup mesh_modifiers

@brief Implementations of the MeshModifier interface

*/

// Documentation for tool path planners
/**
@addtogroup tool_path_planners

@brief Implementations of the ToolPathPlanner interface

*/

// Documentation for raster planners
/**
@addtogroup raster_planners

@brief Implementations of the RasterPlanner interface and associated components

@details A RasterPlanner is a subclass of ToolPathPlanner with the following general properties:

- Generally >1 raster
- Generally covers the whole supplied surface
- Lines are parallel
- Repeating pattern of lines
- All points lie on surface
- All points have same orientation
- Waypoints in rasters are consistently and spatially ordered
- Segments are consistently and spatially ordered in ToolPath
- ToolPaths are in order in top-level structure

@image html docs/raster_path.png

*/

// Documentation for edge planners
/**
@addtogroup edge_planners

@brief Implementations of the EdgePlanner interface

@details An EdgePlanner is a subclass of ToolPathPlanner with the following general properties:

- A list of closed-loop edges around parts
- Segments in sequential order in ToolPath
- ToolPaths ordered by length of closed loop, with longest first
- All loops start near some point (e.g. all in top left corner)
- All loops process in same direction

@image html docs/edge_path.png

*/

// Documentation for direction generators
/**
@addtogroup direction_generators

@brief Implementations of the DirectionGenerator interface

*/

// Documentation for origin generators
/**
@addtogroup origin_generators

@brief Implementations of the OriginGenerator interface

*/

// Documentation for tool path modifiers
/**
@addtogroup tool_path_modifiers

@brief Implementations of the ToolPathModifier interface

*/

// Documentation for GUI interfaces
/**
@addtogroup gui_interfaces

@brief Abstract interfaces for the various components of the GUI

@details The GUI package provides two template interfaces for providing UI support: BaseWidget and WidgetPlugin.

@par
BaseWidget is a Qt widget that uses its UI elements to produce an instance of one of the tool path planning concept @ref
interfaces. Each implementation of a tool path planning concept should provide its own widget such that it can be used
in the GUI application.

@par
WidgetPlugin is the base class for a plugin that can be dynamically loaded at run-time to provide widgets to populate
the GUI. The GUI itself finds available WidgetPlugin classes (via the `NOETHER_PLUGIN_LIBS` environment variable), loads
them, and uses them to produce widgets to populate the various pages of the GUI. Just as each of the tool path planning
concept @ref interfaces should provide a BaseWidget to configure itself, each BaseWidget should also have a
corresponding WidgetPlugin that allows it to be loaded into the GUI.

@par
See @ref page_customization for more information about how to provide a custom tool path planning elements.

@par
The image below shows the architecture of the GUI components.

@image html static/architecture_gui.png

*/

// Documentation for the GUI widget interfaces
/**
@addtogroup gui_interfaces_widgets

@brief Instantitations of the BaseWidget interface

*/

// Documentation for the GUI plugin interfaces
/**
@addtogroup gui_interfaces_plugins

@brief Instantitations of the WidgetPlugin interface

*/

}  // namespace noether
