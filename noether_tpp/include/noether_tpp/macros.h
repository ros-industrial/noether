#pragma once

#define FWD_DECLARE_YAML_STRUCTS()                                                                                     \
  namespace YAML                                                                                                       \
  {                                                                                                                    \
  class Node;                                                                                                          \
                                                                                                                       \
  template <typename T>                                                                                                \
  struct convert;                                                                                                      \
                                                                                                                       \
  template <typename T, typename S>                                                                                    \
  struct as_if;                                                                                                        \
                                                                                                                       \
  }  // namespace YAML

#define FWD_DECLARE_YAML_CONVERT(TYPE)                                                                                 \
  namespace YAML                                                                                                       \
  {                                                                                                                    \
  class Node;                                                                                                          \
                                                                                                                       \
  template <typename T>                                                                                                \
  struct convert;                                                                                                      \
                                                                                                                       \
  template <>                                                                                                          \
  struct convert<TYPE>                                                                                                 \
  {                                                                                                                    \
    using T = TYPE;                                                                                                    \
    static Node encode(const T& val);                                                                                  \
    static bool decode(const Node& node, T& val);                                                                      \
  };                                                                                                                   \
                                                                                                                       \
  }  // namespace YAML

#define DECLARE_YAML_FRIEND_CLASSES(TYPE)                                                                              \
  friend class YAML::convert<TYPE>;                                                                                    \
  friend class YAML::as_if<TYPE, void>;
