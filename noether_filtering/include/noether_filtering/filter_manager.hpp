/*
 * filter_manager.cpp
 *
 *  Created on: Oct 14, 2019
 *      Author: jrgnicho
 */
#ifndef NOETHER_FILTERING_FILTER_MANAGER_HPP_
#define NOETHER_FILTERING_FILTER_MANAGER_HPP_

#include "noether_filtering/filter_manager.h"
#include "noether_filtering/utils.h"
#include <boost/format.hpp>
#include <XmlRpcException.h>

static const std::string CLOUD_PLUGINS_LIBRARY = "libnoether_filtering_cloud_filter_plugins.so";
static const std::string MESH_PLUGINS_LIBRARY = "libnoether_filtering_mesh_filter_plugins.so";

namespace noether_filtering
{

bool loadFilterInfos(XmlRpc::XmlRpcValue mesh_filter_configs,std::vector<FilterInfo>& filter_infos)
{
  if(mesh_filter_configs.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    CONSOLE_BRIDGE_logError("The Mesh Filter Manager Configuration is not an array of filters");
    return false;
  }

  for(int i = 0; i < mesh_filter_configs.size(); i++)
  {
    FilterInfo mi;
    XmlRpc::XmlRpcValue mi_config = mesh_filter_configs[i];
    mi.type_name = static_cast<std::string>(mi_config[config_field_names::TYPE_NAME]);
    mi.name = static_cast<std::string>(mi_config[config_field_names::NAME]);
    mi.config = mi_config[config_field_names::CONFIG];
    filter_infos.push_back(std::move(mi));
  }
  return !filter_infos.empty();
}

template<class F>
FilterManager<F>::FilterManager()
  : continue_on_failure_(false)
  , filter_loader_(true)
{
  try
  {
    filter_loader_.loadLibrary(CLOUD_PLUGINS_LIBRARY);
    filter_loader_.loadLibrary(MESH_PLUGINS_LIBRARY);
  }
  catch (const class_loader::LibraryLoadException &ex)
  {
    CONSOLE_BRIDGE_logError(ex.what());
  }

  auto print_available_filters = [](const std::vector<std::string>& filters) {
    std::string out = "Available classes:";
    for (const std::string &f : filters)
    {
      out += "\n\t\t" + f;
    }
    CONSOLE_BRIDGE_logInform("%s", out.c_str());
  };

  print_available_filters(filter_loader_.getAvailableClassesForLibrary<FilterBase<F>>(CLOUD_PLUGINS_LIBRARY));
  print_available_filters(filter_loader_.getAvailableClassesForLibrary<FilterBase<F>>(MESH_PLUGINS_LIBRARY));
}

template<class F>
bool FilterManager<F>::init(XmlRpc::XmlRpcValue config)
{
  using namespace config_field_names;

  // checking config fields
  const std::vector<std::string> REQUIRED_FIELDS = {CONTINUE_ON_FAILURE, FILTERS};

  auto check_fn = [&config](const std::string &f) {
    if (!config.hasMember(f))
    {
      CONSOLE_BRIDGE_logError("The '%s' field was not found in the %s config",
                              f.c_str(),
                              utils::getClassName<FilterManager<F>>().c_str());
      return false;
    }
    return true;
  };

  if(!std::all_of(REQUIRED_FIELDS.begin(), REQUIRED_FIELDS.end(), check_fn))
  {
    return false;
  }

  // load manager parameters
  try
  {
    continue_on_failure_ = static_cast<bool>(config[CONTINUE_ON_FAILURE]);
  }
  catch (const XmlRpc::XmlRpcException &ex)
  {
    CONSOLE_BRIDGE_logWarn("Failed to parse '%s' parameter: '%s'", CONTINUE_ON_FAILURE.c_str(), ex.getMessage().c_str());
    continue_on_failure_ = false;
  }

  // load filter infos
  std::vector<FilterInfo> filter_infos;
  XmlRpc::XmlRpcValue filter_configs = config[FILTERS];
  if(!loadFilterInfos(filter_configs, filter_infos))
  {
    CONSOLE_BRIDGE_logError("Failed to load mesh filters");
    return false;
  }

  // instantiating filters
  for (FilterInfo &fi : filter_infos)
  {
    if (filters_map_.count(fi.name) > 0)
    {
      CONSOLE_BRIDGE_logError("The %s plugin '%s' has already been added",
                              utils::getClassName<F>().c_str(),
                              fi.name.c_str());
      return false;
    }

    typename class_loader::ClassLoader::UniquePtr<FilterBase<F>> plugin;
    try
    {
      plugin = filter_loader_.createUniqueInstance<FilterBase<F>>(fi.type_name);
      if(!plugin->configure(fi.config))
      {
        CONSOLE_BRIDGE_logError("%s plugin '%s' failed to load configuration",utils::getClassName<F>().c_str(),
                  fi.name.c_str());
        return false;
      }

      // storing plugin
      filters_loaded_.push_back(fi.name);
      filters_map_.insert(std::make_pair(fi.name, std::move(plugin)));
    }
    catch(const class_loader::ClassLoaderException& ex)
    {
      CONSOLE_BRIDGE_logError("%s plugin '%s' could not be created",utils::getClassName<F>().c_str(),
                fi.name.c_str());
      return false;
    }
  }

  return true;
}

template<class F>
bool FilterManager<F>::applyFilters(const F& input, F& output, std::string& err_msg)
{
  // applying all loaded filters
  return applyFilters(filters_loaded_,input, output, err_msg);
}

template<class F>
bool FilterManager<F>::applyFilters(const std::vector<std::string>& filters, const F& input, F& output,
                                    std::string& err_msg)
{
  std::vector<std::string> selected_filters = filters;
  if(selected_filters.empty())
  {
    CONSOLE_BRIDGE_logWarn("%s received empty list of filters, using all filters loaded", utils::getClassName<FilterManager<F>>().c_str());
    std::copy(filters_loaded_.begin(), filters_loaded_.end(), std::back_inserter(selected_filters));
  }

  // check all filters exists
  auto check_fn = [this, &err_msg](const std::string &f) {
    if (filters_map_.count(f) == 0)
    {
      err_msg = boost::str(boost::format("The mesh filter %s was not found") % f);
      CONSOLE_BRIDGE_logError("%s", err_msg.c_str());
      return false;
    }
    return true;
  };

  if(!std::all_of(filters.begin(), filters.end(), check_fn))
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

#endif // NOETHER_FILTERING_FILTER_MANAGER_HPP_
