/*
 * filter_manager.cpp
 *
 *  Created on: Oct 14, 2019
 *      Author: jrgnicho
 */

#include <noether_filtering/filter_group.hpp>
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

struct MeshFilterInfo
{
  std::string type_name;      /** @brief the filter type */
  std::string name;           /** @brief an alias for the filter */
  XmlRpc::XmlRpcValue config; /** @brief the filter configuration */
};

static bool loadFilterInfos(XmlRpc::XmlRpcValue mesh_filter_configs,std::vector<MeshFilterInfo>& filter_infos)
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

template<class F>
FilterGroup<F>::FilterGroup():
  continue_on_failure_(false)
{

}

template<class F>
FilterGroup<F>::~FilterGroup()
{

}

template<class F>
bool FilterGroup<F>::init(XmlRpc::XmlRpcValue config)
{
  using namespace XmlRpc;
  using namespace config_field_names;
  using FilterLoader = pluginlib::ClassLoader< FilterT >;
  using PluginPtr = std::unique_ptr<FilterT>;
  filter_loader_ = std::make_shared< FilterLoader>(PACKAGE_NAME,utils::getClassName<F>());

  // checking config fields
  const std::vector<std::string> REQUIRED_FIELDS = {CONTINUE_ON_FAILURE, MESH_FILTERS};
  if(!std::all_of(REQUIRED_FIELDS.begin(), REQUIRED_FIELDS.end(),[&](const std::string& f){
    if(!config.hasMember(f))
    {
      CONSOLE_BRIDGE_logError("The '%s' field was not found in the %s config",f.c_str(),
                              utils::getClassName< FilterGroup<F> >().c_str());
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
      CONSOLE_BRIDGE_logError("The %s plugin '%s' has already been added",utils::getClassName<F>().c_str(),
                mi.name.c_str());
      return false;
    }

    PluginPtr plugin;
    try
    {
      plugin.reset(filter_loader_->createUnmanagedInstance(mi.type_name));
      if(!plugin->configure(mi.config))
      {
        CONSOLE_BRIDGE_logError("%s plugin '%s' failed to load configuration",utils::getClassName<F>().c_str(),
                  mi.name.c_str());
        return false;
      }

      // storing plugin
      filters_loaded_.push_back(mi.name);
      filters_map_.insert(std::make_pair(mi.name,std::move(plugin)));
    }
    catch(pluginlib::PluginlibException& e)
    {
      CONSOLE_BRIDGE_logError("%s plugin '%s' could not be created",utils::getClassName<F>().c_str(),
                mi.name.c_str());
      return false;
    }
  }

  return true;
}

template<class F>
bool FilterGroup<F>::applyFilters(const F& input, F& output, std::string& err_msg)
{
  // applying all loaded filters
  return applyFilters(filters_loaded_,input, output, err_msg);
}

template<class F>
bool FilterGroup<F>::applyFilters(const std::vector<std::string>& filters, const F& input, F& output,
                                    std::string& err_msg)
{
  std::vector<std::string> selected_filters = filters;
  if(selected_filters.empty())
  {
    CONSOLE_BRIDGE_logWarn("%s received empty list of filters, using all filters loaded", utils::getClassName< FilterGroup<F> >().c_str());
    std::copy(filters_loaded_.begin(), filters_loaded_.end(), std::back_inserter(selected_filters));
  }

  // check all filters exists
  if(!std::all_of(selected_filters.begin(), selected_filters.end(),[&](const std::string& f){
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
  for(const std::string& fname : selected_filters)
  {
    bool current_filter_succeeded = filters_map_[fname]->filter(input,output);
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
