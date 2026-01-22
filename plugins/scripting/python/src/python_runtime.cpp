#include "rkg/log.h"
#include "rkg/plugin_api.h"
#include "rkg_script/script_runtime.h"

namespace {

class PythonRuntime final : public rkg::script::IScriptRuntime {
 public:
  bool init(rkg::HostContext* host) override {
    (void)host;
#if RKG_HAVE_PYTHON
    rkg::log::info("script:python init");
    return true;
#else
    rkg::log::warn("script:python not available (stub)");
    return false;
#endif
  }

  bool load_script(const std::string& source_or_path) override {
    (void)source_or_path;
    return false;
  }

  void tick(float dt_seconds) override {
    (void)dt_seconds;
  }

  void call(const std::string& fn, const std::vector<std::string>& args) override {
    (void)fn;
    (void)args;
  }

  void bind_native(const rkg::script::NativeBinding& binding) override {
    (void)binding;
  }
};

PythonRuntime g_runtime;

bool py_plugin_init(void* host) {
  return g_runtime.init(static_cast<rkg::HostContext*>(host));
}

void py_plugin_shutdown() {}

void py_plugin_update(float dt) {
  g_runtime.tick(dt);
}

rkg::RkgPluginApi g_api{
    rkg::kRkgPluginApiVersion,
    "script_python",
    rkg::PluginType::Scripting,
    0,
    &py_plugin_init,
    &py_plugin_shutdown,
    &py_plugin_update,
    nullptr};
} // namespace

extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_script_python(uint32_t host_api_version) {
  if (host_api_version != rkg::kRkgPluginApiVersion) {
    return nullptr;
  }
  return &g_api;
}

#if defined(RKG_BUILD_DYNAMIC_PLUGIN)
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api(uint32_t host_api_version) {
  return rkg_plugin_get_api_script_python(host_api_version);
}
#endif
