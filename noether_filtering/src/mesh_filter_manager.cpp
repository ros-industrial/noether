/*
 * filter_manager.cpp
 *
 *  Created on: Oct 10, 2019
 *      Author: jrgnicho
 */

#include "noether_filtering/mesh_filter_manager.h"
#include "noether_filtering/utils.h"

static const std::string PACKAGE_NAME = "noether_filtering";

namespace config_field_names
{
  static const std::string CONTINUE_ON_FAILURE = "continue_on_failure";
  static const std::string MESH_FILTERS = "mesh_filters";
  static const std::string TYPE_NAME = "type";
  static const std::string NAME = "name";
  static const std::string CONFIG = "config";
}

namespace noether_filtering
{

bool loadFilterInfos(XmlRpc::XmlRpcValue mesh_filter_configs,std::vector<MeshFilterInfo>& filter_infos)
{
  using namespace XmlRpc;
  using namespace config_field_names;

  if(mesh_filter_configs.getType() != XmlRpcValue::TypeArray)
  {
    CONSOLE_BRIDGE_logError("The Mesh Filter Manager Configuration is not an array of filters");
    return false;
  }

  for(int i = 0; i < mesh_filter_configs.size(); i++)
  {
    MeshFilterInfo mi;
    XmlRpcValue mi_config = mesh_filter_configs[i];
    mi.type_name = static_cast<std::string>(mi_config[TYPE_NAME]);
    mi.name = static_cast<std::string>(mi_config[NAME]);
    mi.config = mi_config[CONFIG];
    filter_infos.push_back(std::move(mi));
  }
  return !filter_infos.empty();
}

MeshFilterManager::MeshFilterManager():
    continue_on_failure_(false)
{

}

MeshFilterManager::~MeshFilterManager()
{

}

bool MeshFilterManager::init(XmlRpc::XmlRpcValue config)
{
  using namespace XmlRpc;
  using namespace config_field_names;
  using FilterLoader = pluginlib::ClassLoader<MeshFilterBase>;
  using PluginPtr = std::unique_ptr<MeshFilterBase>;
  filter_loader_ = std::make_shared<FilterLoader>(PACKAGE_NAME,utils::getClassName<MeshFilterBase>());

  // checking config fields
  const std::vector<std::string> REQUIRED_FIELDS = {CONTINUE_ON_FAILURE, MESH_FILTERS};
  if(!std::all_of(REQUIRED_FIELDS.begin(), REQUIRED_FIELDS.end(),[&](const std::string& f){
    if(!config.hasMember(f))
    {
      CONSOLE_BRIDGE_logError("The '%s' field was not found in the %s config",f.c_str(),
                              utils::getClassName<MeshFilterManager>().c_str());
      return false;
    }
    return true;
  }))
  {
    return false;
  }

  // load manager parameters
  continue_on_failure_ = static_cast<bool>(config[CONTINUE_ON_FAILURE]);

  // load filter infos
  std::vector<MeshFilterInfo> filter_infos;
  XmlRpcValue mesh_filter_configs = config[MESH_FILTERS];
  if(!loadFilterInfos(mesh_filter_configs, filter_infos))
  {
    CONSOLE_BRIDGE_logError("Failed to load mesh filters");
    return false;
  }

  // instantiating filters
  for(MeshFilterInfo& mi : filter_infos)
  {
    if(filters_map_.count(mi.name) > 0)
    {
      CONSOLE_BRIDGE_logError("The %s plugin '%s' has already been added",utils::getClassName<MeshFilterBase>().c_str(),
                mi.name.c_str());
      return false;
    }

    PluginPtr plugin;
    try
    {
      plugin.reset(filter_loader_->createUnmanagedInstance(mi.type_name));
      if(!plugin->configure(mi.config))
      {
        CONSOLE_BRIDGE_logError("%s plugin '%s' failed to load configuration",utils::getClassName<MeshFilterBase>().c_str(),
                  mi.name.c_str());
        return false;
      }

      // storing plugin
      filters_loaded_.push_back(mi.name);
      filters_map_.insert(std::make_pair(mi.name,std::move(plugin)));
    }
    catch(pluginlib::PluginlibException& e)
    {
      CONSOLE_BRIDGE_logError("%s plugin '%s' could not be created",utils::getClassName<MeshFilterBase>().c_str(),
                mi.name.c_str());
      return false;
    }
  }

  return true;
}

bool MeshFilterManager::applyFilters(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out, std::string& err_msg)
{
  // applying all loaded filters
  return applyFilters(filters_loaded_,mesh_in, mesh_out, err_msg);
}

bool MeshFilterManager::applyFilters(const std::vector<std::string>& filters, const pcl::PolygonMesh& mesh_in,
                                     pcl::PolygonMesh& mesh_out, std::string& err_msg)
{
  // check all filters exists
  if(!std::all_of(filters.begin(), filters.end(),[&](const std::string& f){
    if (filters_map_.count(f) == 0)
    {
      err_msg = boost::str(boost::format("The mesh filter %s was not found") % f);
      CONSOLE_BRIDGE_logError("%s", err_msg.c_str());
      return false;
    }
    return true;
  }))
  {
    return false;
  }

  // apply filters
  bool success = false;
  for(const std::string& fname : filters)
  {
    bool current_filter_succeeded = filters_map_[fname]->filter(mesh_in,mesh_out);
    if(!current_filter_succeeded)
    {
      err_msg = boost::str(boost::format("The mesh filter %s failed") % fname);
      CONSOLE_BRIDGE_logError("%s", err_msg.c_str());
    }

    if(!current_filter_succeeded && !continue_on_failure_)
    {
      return false;
    }

    success |= current_filter_succeeded;
  }
  return success;
}

} /* namespace noether_filtering */
