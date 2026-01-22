#include "rkg/plugin_host.h"

#include "rkg/log.h"

namespace rkg {

void PluginHost::register_plugin(RkgPluginApi* api) {
  if (!api || !api->name) {
    return;
  }
  plugins_[api->name] = api;
}

bool PluginHost::init_all(void* host_context) {
  bool ok = true;
  for (const auto& kv : plugins_) {
    auto* api = kv.second;
    if (api && api->init) {
      if (!api->init(host_context)) {
        log::warn(std::string("plugin init failed: ") + api->name);
        ok = false;
      }
    }
  }
  return ok;
}

void PluginHost::shutdown_all() {
  for (const auto& kv : plugins_) {
    auto* api = kv.second;
    if (api && api->shutdown) {
      api->shutdown();
    }
  }
}

void PluginHost::update_all(float dt_seconds) {
  for (const auto& kv : plugins_) {
    auto* api = kv.second;
    if (api && api->update) {
      api->update(dt_seconds);
    }
  }
}

bool PluginHost::init_plugin(const std::string& name, void* host_context) {
  auto* api = find_by_name(name);
  if (!api || !api->init) {
    return false;
  }
  return api->init(host_context);
}

void PluginHost::shutdown_plugin(const std::string& name) {
  auto* api = find_by_name(name);
  if (api && api->shutdown) {
    api->shutdown();
  }
}

void PluginHost::update_plugin(const std::string& name, float dt_seconds) {
  auto* api = find_by_name(name);
  if (api && api->update) {
    api->update(dt_seconds);
  }
}

RkgPluginApi* PluginHost::find_by_name(const std::string& name) const {
  auto it = plugins_.find(name);
  if (it == plugins_.end()) {
    return nullptr;
  }
  return it->second;
}

std::vector<RkgPluginApi*> PluginHost::list_by_type(PluginType type) const {
  std::vector<RkgPluginApi*> out;
  for (const auto& kv : plugins_) {
    if (kv.second && kv.second->type == type) {
      out.push_back(kv.second);
    }
  }
  return out;
}

} // namespace rkg
