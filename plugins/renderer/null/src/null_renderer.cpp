#include "rkg/host_context.h"
#include "rkg/log.h"
#include "rkg/plugin_api.h"

namespace {
rkg::HostContext* g_ctx = nullptr;

bool null_init(void* host) {
  g_ctx = static_cast<rkg::HostContext*>(host);
  rkg::log::info("renderer:null init");
  return true;
}

void null_shutdown() {
  rkg::log::info("renderer:null shutdown");
  g_ctx = nullptr;
}

void null_update(float) {
  (void)g_ctx;
}

rkg::RkgPluginApi g_api{
    rkg::kRkgPluginApiVersion,
    "renderer_null",
    rkg::PluginType::Renderer,
    static_cast<uint32_t>(rkg::RendererCaps::None),
    &null_init,
    &null_shutdown,
    &null_update,
    nullptr};
} // namespace

extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_renderer_null(uint32_t host_api_version) {
  if (host_api_version != rkg::kRkgPluginApiVersion) {
    return nullptr;
  }
  return &g_api;
}

#if defined(RKG_BUILD_DYNAMIC_PLUGIN)
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api(uint32_t host_api_version) {
  return rkg_plugin_get_api_renderer_null(host_api_version);
}
#endif
