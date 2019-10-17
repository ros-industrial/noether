#ifndef NOETHER_FILTERING_FILTER_MANAGER_HPP
#define NOETHER_FILTERING_FILTER_MANAGER_HPP

#include "noether_filtering/filter_manager.h"
#include "noether_filtering/utils.h"
#include <XmlRpcException.h>
#include <console_bridge/console.h>

static const std::string DEFAULT_FILTER_CHAIN_NAME = "Default";

namespace noether_filtering
{

template<typename T>
bool FilterManager<T>::init(XmlRpc::XmlRpcValue config)
{
  using namespace config_fields;

  if(!config.hasMember(manager::FILTER_GROUPS))
  {
    CONSOLE_BRIDGE_logError("The '%s' field was not found, %s failed to load configuration",
                            manager::FILTER_GROUPS.c_str(),
                            utils::getClassName<decltype(*this)>().c_str());
    return false;
  }

  XmlRpc::XmlRpcValue filter_groups_config = config[manager::FILTER_GROUPS];
  if(filter_groups_config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    CONSOLE_BRIDGE_logError("The '%s' field is not an array, %s failed to load configuration",
                            manager::FILTER_GROUPS.c_str(),
                            utils::getClassName<decltype(*this)>().c_str());
    return false;
  }

  for(int i = 0; i < filter_groups_config.size(); i++)
  {
    try
    {
      // Create the filter group object
      std::shared_ptr<FilterGroup<T>> filter_group = std::make_shared<FilterGroup<T>>();

      // Load the group configuration
      XmlRpc::XmlRpcValue group_config = filter_groups_config[i];
      std::string group_name = static_cast<std::string>(group_config[group::GROUP_NAME]);

      if(filter_groups_map_.find(group_name) != filter_groups_map_.end())
      {
        CONSOLE_BRIDGE_logError("The filter group '%s' already exists in %s",
                                group_name.c_str(),
                                utils::getClassName<decltype(*this)>().c_str());
        return false;
      }

      if(!filter_group->init(group_config))
      {
        CONSOLE_BRIDGE_logError("Failed to initialize filter group '%s'", group_name.c_str());
        return false;
      }

      filter_groups_map_.insert(std::make_pair(group_name, std::move(filter_group)));
      CONSOLE_BRIDGE_logInform("%s loaded filter group '%s'",
                               utils::getClassName<decltype(*this)>().c_str(),
                               group_name.c_str());
    }
    catch(XmlRpc::XmlRpcException& e)
    {
      CONSOLE_BRIDGE_logError("Failure while loading %s configuration, error msg: '%s'",
                              utils::getClassName<decltype(*this)>().c_str(),
                              e.getMessage().c_str());
      return false;
    }
  }

  return true;
}

template<typename T>
std::shared_ptr<FilterGroup<T>> FilterManager<T>::getFilterGroup(const std::string& name)
{
  std::string n = name.empty() ? DEFAULT_FILTER_CHAIN_NAME : name;
  if(filter_groups_map_.find(n) == filter_groups_map_.end())
  {
    CONSOLE_BRIDGE_logError("Filter chain '%s' was not found", n.c_str());
    return nullptr;
  }
  return filter_groups_map_[n];
}

} // namespace noether_filtering

#endif // NOETHER_FILTERING_FILTER_MANAGER_HPP
