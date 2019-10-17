/*
 * filter_manager.cpp
 *
 *  Created on: Oct 14, 2019
 *      Author: jrgnicho
 */
#ifndef NOETHER_FILTERING_FILTER_GROUP_HPP_
#define NOETHER_FILTERING_FILTER_GROUP_HPP_

#include "noether_filtering/filter_group.h"
#include "noether_filtering/utils.h"
#include <boost/format.hpp>
#include <XmlRpcException.h>

static const std::string FILTER_PLUGINS_LIBRARY = "libnoether_filtering_filter_plugins.so";

struct FilterInfo
{
  std::string type_name;      /** @brief the filter type */
  std::string name;           /** @brief an alias for the filter */
  XmlRpc::XmlRpcValue config; /** @brief the filter configuration */
};

namespace
{
bool loadFilterInfos(XmlRpc::XmlRpcValue filter_configs, std::vector<FilterInfo>& filter_infos)
{
  if(filter_configs.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    CONSOLE_BRIDGE_logError("The filter group configuration is not an array of filters");
    return false;
  }

  for(int i = 0; i < filter_configs.size(); i++)
  {
    using namespace noether_filtering::config_fields;
    FilterInfo fi;
    XmlRpc::XmlRpcValue fi_config = filter_configs[i];
    fi.type_name = static_cast<std::string>(fi_config[filter::TYPE_NAME]);
    fi.name = static_cast<std::string>(fi_config[filter::NAME]);
    fi.config = fi_config[filter::CONFIG];
    filter_infos.push_back(std::move(fi));
  }
  return !filter_infos.empty();
}
} // namespace anonymous

namespace noether_filtering
{

template<class F>
FilterGroup<F>::FilterGroup()
  : continue_on_failure_(false)
  , filter_loader_(FILTER_PLUGINS_LIBRARY)
{
  std::vector<std::string> filters = filter_loader_.getAvailableClasses<FilterBase<F>>();
  std::string out = "Available classes:";
  for (const std::string &f : filters)
  {
    out += "\n\t\t" + f;
  }
  CONSOLE_BRIDGE_logInform("%s", out.c_str());
}

template<class F>
bool FilterGroup<F>::init(XmlRpc::XmlRpcValue config)
{
  using namespace config_fields::group;

  // checking config fields
  const std::vector<std::string> REQUIRED_FIELDS = {CONTINUE_ON_FAILURE, FILTERS};

  auto check_fn = [&config](const std::string &f) {
    if (!config.hasMember(f))
    {
      CONSOLE_BRIDGE_logError("The '%s' field was not found in the %s config",
                              f.c_str(),
                              utils::getClassName<FilterGroup<F>>().c_str());
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
    CONSOLE_BRIDGE_logError("Failed to load filters");
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
    CONSOLE_BRIDGE_logWarn("%s received empty list of filters, using all filters loaded", utils::getClassName<FilterGroup<F>>().c_str());
    std::copy(filters_loaded_.begin(), filters_loaded_.end(), std::back_inserter(selected_filters));
  }

  // check all filters exists
  auto check_fn = [this, &err_msg](const std::string &f) {
    if (filters_map_.count(f) == 0)
    {
      err_msg = boost::str(boost::format("The filter %s was not found") % f);
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
      err_msg = boost::str(boost::format("The filter %s failed") % fname);
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

#endif // NOETHER_FILTERING_FILTER_GROUP_HPP_
