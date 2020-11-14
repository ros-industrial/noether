#ifndef NOETHER_FILTERING_CLOUD_IMPL_CROP_BOX_FILTER_HPP
#define NOETHER_FILTERING_CLOUD_IMPL_CROP_BOX_FILTER_HPP

#include "noether_filtering/cloud/crop_box_filter.h"
#include "noether_filtering/utils.h"
#include <XmlRpcException.h>
#include <console_bridge/console.h>
#include <pcl/filters/crop_box.h>

namespace
{
bool fromXmlRpc(XmlRpc::XmlRpcValue value, Eigen::Ref<Eigen::Vector4f> out)
{
  try
  {
    out.x() = static_cast<double>(value["x"]);
    out.y() = static_cast<double>(value["y"]);
    out.z() = static_cast<double>(value["z"]);
  }
  catch (const XmlRpc::XmlRpcException& ex)
  {
    CONSOLE_BRIDGE_logError("%s", ex.getMessage().c_str());
    return false;
  }
  return true;
}

bool fromXmlRpc(XmlRpc::XmlRpcValue value, Eigen::Affine3f& out)
{
  out = Eigen::Affine3f::Identity();
  try
  {
    out.translation().x() = static_cast<double>(value["x"]);
    out.translation().y() = static_cast<double>(value["y"]);
    out.translation().z() = static_cast<double>(value["z"]);

    Eigen::AngleAxisf rx(static_cast<double>(value["rx"]), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf ry(static_cast<double>(value["ry"]), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rz(static_cast<double>(value["rz"]), Eigen::Vector3f::UnitZ());
    out.rotate(rx * ry * rz);
  }
  catch (const XmlRpc::XmlRpcException& ex)
  {
    CONSOLE_BRIDGE_logError("%s", ex.getMessage().c_str());
    return false;
  }
  return true;
}

}  // namespace

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
const std::string CropBoxFilter<PointT>::MAX = "max";

template <typename PointT>
const std::string CropBoxFilter<PointT>::MIN = "min";

template <typename PointT>
const std::string CropBoxFilter<PointT>::TRANSFORM = "transform";

template <typename PointT>
const std::string CropBoxFilter<PointT>::CROP_OUTSIDE = "crop_outside";

template <typename PointT>
bool CropBoxFilter<PointT>::configure(XmlRpc::XmlRpcValue value)
{
  std::string error;
  if (!value.hasMember(MIN))
    error += MIN + ", ";
  if (!value.hasMember(MAX))
    error += MAX + ", ";
  if (!value.hasMember(TRANSFORM))
    error += TRANSFORM + ", ";

  if (!error.empty())
  {
    CONSOLE_BRIDGE_logError("Filter configuration missing required parameters: %s", error.c_str());
    return false;
  }

  // Required parameter(s)
  try
  {
    bool success = true;
    success &= fromXmlRpc(value[MIN], params.min_pt);
    success &= fromXmlRpc(value[MAX], params.max_pt);
    success &= fromXmlRpc(value[TRANSFORM], params.transform);
    if (!success)
    {
      return false;
    }
  }
  catch (const XmlRpc::XmlRpcException& ex)
  {
    CONSOLE_BRIDGE_logError("Failed to load required parameter(s) for pass through filter: '%s'",
                            ex.getMessage().c_str());
    return false;
  }

  // Optional parameter(s)
  if (value.hasMember(CROP_OUTSIDE))
  {
    try
    {
      params.crop_outside = static_cast<bool>(value[CROP_OUTSIDE]);
    }
    catch (const XmlRpc::XmlRpcException& ex)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load optional parameter(s) for crop box filter: '%s'", ex.getMessage().c_str());
    }
  }

  return true;
}

template <typename PointT>
bool CropBoxFilter<PointT>::filter(const T& input, T& output)
{
  // Create a shared pointer to the input object with a "destructor" function that does not delete the raw pointer
  auto cloud = boost::shared_ptr<const T>(&input, [](const T*) {});

  // Set the parameters
  pcl::CropBox<PointT> f(params.crop_outside);
  f.setMin(params.min_pt);
  f.setMax(params.max_pt);
  f.setTransform(params.transform);

  f.setInputCloud(cloud);
  f.filter(output);

  return true;
}

template <typename PointT>
std::string CropBoxFilter<PointT>::getName() const
{
  return utils::getClassName<decltype(this)>();
}

}  // namespace cloud
}  // namespace noether_filtering

#define PCL_INSTANTIATE_CropBoxFilter(T) template class PCL_EXPORTS noether_filtering::cloud::CropBoxFilter<T>;

#endif  // NOETHER_FILTERING_CLOUD_IMPL_CROP_BOX_FILTER_HPP
