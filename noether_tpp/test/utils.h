#pragma once

#include <regex>
#include <cxxabi.h>
#include <stdexcept>
#include <gtest/gtest.h>

namespace noether
{
/** @brief Extracts the demangled class name behind the namespace for printing in unit test */
template <typename T>
std::string getClassName(const T& obj)
{
  std::regex re(".*::(.*)");
  std::smatch match;

  int status;
  std::string class_name(abi::__cxa_demangle(typeid(obj).name(), 0, 0, &status));
  if (std::regex_match(class_name, match, re))
    return match[1];
  throw std::runtime_error("Failed to get class name from demangled name");
}

/** @brief Prints name of class for unit test output */
template <typename T>
std::string print(const testing::TestParamInfo<std::shared_ptr<T>> info)
{
  return getClassName(*info.param) + "_" + std::to_string(info.index);
}

}  // namespace noether
