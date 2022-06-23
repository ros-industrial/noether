#include <noether_gui/widgets/plugin_loader_widget.hpp>
#include <noether_gui/plugin_interface.h>

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
template class PluginLoaderWidget<MeshModifierWidgetPlugin>;
template class PluginLoaderWidget<ToolPathModifierWidgetPlugin>;

}  // namespace noether
