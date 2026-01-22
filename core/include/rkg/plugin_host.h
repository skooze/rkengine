#pragma once

#include "rkg/plugin_api.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace rkg {

class PluginHost {
 public:
  void register_plugin(RkgPluginApi* api);
  bool init_all(void* host_context);
  void shutdown_all();
  void update_all(float dt_seconds);

  bool init_plugin(const std::string& name, void* host_context);
  void shutdown_plugin(const std::string& name);
  void update_plugin(const std::string& name, float dt_seconds);

  RkgPluginApi* find_by_name(const std::string& name) const;
  std::vector<RkgPluginApi*> list_by_type(PluginType type) const;

 private:
  std::unordered_map<std::string, RkgPluginApi*> plugins_;
};

} // namespace rkg
