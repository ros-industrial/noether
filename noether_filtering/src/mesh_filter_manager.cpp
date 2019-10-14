/*
 * filter_manager.cpp
 *
 *  Created on: Oct 10, 2019
 *      Author: jrgnicho
 */

#include "noether_filtering/mesh_filter_manager.h"
#include "noether_filtering/utils.h"


static const std::string DEFAULT_FILTER_CHAIN_NAME = "Default";
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
 * FiltersManager:
 *
 * @param config
 * @return True on success, false otherwise
 */
bool MeshFilterManager::init(XmlRpc::XmlRpcValue config)
{
  CONSOLE_BRIDGE_logError("%s not implemented yet",__PRETTY_FUNCTION__);
  return false;
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
