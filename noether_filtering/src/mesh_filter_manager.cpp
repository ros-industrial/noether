/*
 * filter_manager.cpp
 *
 *  Created on: Oct 10, 2019
 *      Author: jrgnicho
 */

#include <XmlRpcException.h>
#include "noether_filtering/mesh_filter_manager.h"
#include "noether_filtering/utils.h"


static const std::string DEFAULT_FILTER_CHAIN_NAME = "Default";

namespace config_field_names
{
  static const std::string FILTER_GROUPS = "filter_groups";
  static const std::string GROUP_NAME = "group_name";
}

namespace noether_filtering
{


MeshFilterManager::MeshFilterManager()
{

}

MeshFilterManager::~MeshFilterManager()
{

}

/**
 * @details loads several filter chain configurations. The yaml structure is expected to be as follows:
 * filter_groups:
 * - group_name: DEFAULT
 *   continue_on_failure: False
 *   mesh_filters:
 *   - type: DemoFilter1
 *     name: demo_filter_1
 *     config:
 *      param1: 20
 *      param2: 'optimize'
 *      .
 *      .
 *      .
 *   - type: DemoFilter2
 *     name: demo_filter_2
 *     config:
 *      x: 1.0
 *      y: 3.5
 * - group_name: GROUP_1
 *   continue_on_failure: False
 *   mesh_filters:
 *   - type: DemoFilterX
 *     name: demo_filter_x
 *     config:
 *      param_x: 20
 *      .
 *      .
 *      .
 *   - type: DemoFilterY
 *     name: demo_filter_y
 *     config:
 *      a: True
 *      b: 20
 *   .
 *   .
 *   .FI
 *
 * @param config The configuration
 * @return True on success, false otherwise
 */
bool MeshFilterManager::init(XmlRpc::XmlRpcValue config)
{
  using namespace XmlRpc;
  using MeshFilterGroup = FilterGroup<MeshFilterBase>;

  if(!config.hasMember(config_field_names::FILTER_GROUPS))
  {
    CONSOLE_BRIDGE_logError("The '%s' field was not found, %s failed to load configuration",
                            config_field_names::FILTER_GROUPS.c_str(),
                            utils::getClassName<decltype(*this)>().c_str());
    return false;
  }

  XmlRpcValue filter_groups_config = config[config_field_names::FILTER_GROUPS];
  if(filter_groups_config.getType() != XmlRpcValue::TypeArray)
  {
    CONSOLE_BRIDGE_logError("The '%s' field is not an array, %s failed to load configuration",
                            config_field_names::FILTER_GROUPS.c_str(),
                            utils::getClassName<decltype(*this)>().c_str());
    return false;
  }

  for(int i = 0; i < filter_groups_config.size(); i++)
  {
    try
    {
      XmlRpcValue config = filter_groups_config[i];
      std::shared_ptr<MeshFilterGroup> filter_group = std::make_shared<MeshFilterGroup>();
      std::string group_name = static_cast<std::string>(config[config_field_names::GROUP_NAME]);

      if(filter_groups_map_.count(group_name) > 0)
      {
        CONSOLE_BRIDGE_logError("The filter group '%s' already exists in %s",
                                group_name.c_str(),
                                utils::getClassName<decltype(*this)>().c_str());
        return false;
      }

      if(!filter_group->init(config))
      {
        CONSOLE_BRIDGE_logError("Failed to initialize filter group '%s'", group_name.c_str());
        return false;
      }

      filter_groups_map_.insert(std::make_pair(group_name,std::move(filter_group)));
    }
    catch(XmlRpcException& e)
    {
      CONSOLE_BRIDGE_logError("Failure while loading %s configuration, error msg: %s",
                              utils::getClassName<decltype(*this)>().c_str(),
                              e.getMessage().c_str());
      return false;
    }
  }

  return true;
}

std::shared_ptr<FilterGroup<MeshFilterBase> > MeshFilterManager::getFilterGroup(const std::string& name)
{
  std::string n = name.empty() ? DEFAULT_FILTER_CHAIN_NAME : name;
  if(filter_groups_map_.count(n) == 0)
  {
    CONSOLE_BRIDGE_logError("Filter chain '%s' was not found", n.c_str());
    return nullptr;
  }
  return filter_groups_map_[n];
}

} /* namespace noether_filtering */
