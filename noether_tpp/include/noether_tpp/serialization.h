#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>

namespace YAML
{

// Helper function to get a member with error checking
template <typename T>
T getMember(const Node& node, const std::string& key)
{
  if (!node[key])
    throw std::runtime_error("Required member '" + key + "' not found in YAML node");
  return node[key].as<T>();
}

// Serialization for Eigen::Transform<FloatT, 3, Eigen::Isometry>
template <typename FloatT>
struct convert<Eigen::Transform<FloatT, 3, Eigen::Isometry>>
{
  using T = Eigen::Transform<FloatT, 3, Eigen::Isometry>;

  static Node encode(const T& val)
  {
    YAML::Node node;
    node["x"] = val.translation().x();
    node["y"] = val.translation().y();
    node["z"] = val.translation().z();

    Eigen::Quaternion<FloatT> quat(val.rotation());
    node["qw"] = quat.w();
    node["qx"] = quat.x();
    node["qy"] = quat.y();
    node["qz"] = quat.z();

    return node;
  }

  static bool decode(const Node& node, T& val)
  {
    Eigen::Matrix<FloatT, 3, 1> trans;
    trans.x() = getMember<FloatT>(node, "x");
    trans.y() = getMember<FloatT>(node, "y");
    trans.z() = getMember<FloatT>(node, "z");

    Eigen::Quaternion<FloatT> quat;
    quat.w() = getMember<FloatT>(node, "qw");
    quat.x() = getMember<FloatT>(node, "qx");
    quat.y() = getMember<FloatT>(node, "qy");
    quat.z() = getMember<FloatT>(node, "qz");

    val = Eigen::Translation<FloatT, 3>(trans) * quat;
    return true;
  }
};

// Serialization for Eigen::Vector<FloatT, 3, 1>
template <typename FloatT>
struct convert<Eigen::Matrix<FloatT, 3, 1>>
{
  using T = Eigen::Matrix<FloatT, 3, 1>;

  static Node encode(const T& val)
  {
    YAML::Node node;
    node["x"] = val.x();
    node["y"] = val.y();
    node["z"] = val.z();
    return node;
  }

  static bool decode(const Node& node, T& val)
  {
    val.x() = getMember<FloatT>(node, "x");
    val.y() = getMember<FloatT>(node, "y");
    val.z() = getMember<FloatT>(node, "z");
    return true;
  }
};

// Serialization for std::vectors with Eigen-specific allocators (e.g., ToolPath objects)
template <typename T>
struct convert<std::vector<T, Eigen::aligned_allocator<T>>>
{
  static Node encode(const std::vector<T, Eigen::aligned_allocator<T>>& v)
  {
    Node node;
    for (const auto& elem : v)
      node.push_back(elem);

    return node;
  }

  static bool decode(const Node& node, std::vector<T, Eigen::aligned_allocator<T>>& v)
  {
    if (!node.IsSequence())
      return false;

    v.clear();
    for (const auto& elem : node)
      v.push_back(elem.as<T>());

    return true;
  }
};

}  // namespace YAML
