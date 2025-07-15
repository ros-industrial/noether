#include <noether_tpp/plugin_interface.h>
#include <memory>

namespace noether
{
template <typename DerivedT, typename BaseT>
struct PluginImpl : public Plugin<BaseT>
{
  std::unique_ptr<BaseT> create(const YAML::Node& config = {}) const override final
  {
    return std::make_unique<DerivedT>(config.as<DerivedT>());
  }
};

}  // namespace noether
