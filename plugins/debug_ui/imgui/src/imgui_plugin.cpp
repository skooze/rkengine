#include "rkg/plugin_api.h"

#include "rkg/log.h"
#include "rkg_debug_ui/imgui_api.h"

namespace {

bool debug_ui_init(void* host) {
  rkg::debug_ui::set_host_context(host);
  if (!rkg::debug_ui::init_vulkan()) {
    rkg::log::warn("debug_ui: init failed");
    return false;
  }
  rkg::log::info("debug_ui: init");
  return true;
}

void debug_ui_shutdown() {
  rkg::debug_ui::shutdown();
  rkg::log::info("debug_ui: shutdown");
}

void debug_ui_update(float dt) {
  rkg::debug_ui::new_frame(dt);
}

rkg::RkgPluginApi g_api{
    rkg::kRkgPluginApiVersion,
    "debug_ui_imgui",
    rkg::PluginType::DebugUI,
    0,
    &debug_ui_init,
    &debug_ui_shutdown,
    &debug_ui_update,
    nullptr};
} // namespace

extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_debug_ui_imgui(uint32_t host_api_version) {
  if (host_api_version != rkg::kRkgPluginApiVersion) {
    return nullptr;
  }
  return &g_api;
}

#if defined(RKG_BUILD_DYNAMIC_PLUGIN)
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api(uint32_t host_api_version) {
  return rkg_plugin_get_api_debug_ui_imgui(host_api_version);
}
#endif
