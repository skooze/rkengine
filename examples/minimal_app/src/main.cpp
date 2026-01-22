#include "rkg/config.h"
#include "rkg/ecs.h"
#include "rkg/host_context.h"
#include "rkg/log.h"
#include "rkg/paths.h"
#include "rkg/plugin_api.h"
#include "rkg/plugin_host.h"
#include "rkg/instructions.h"
#include "rkg_platform/platform.h"

#include <SDL3/SDL.h>

#if RKG_ENABLE_DATA_YAML
#include <yaml-cpp/yaml.h>
#endif

#include <filesystem>
#include <optional>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;

extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_renderer_null(uint32_t host_api_version);
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_renderer_vulkan(uint32_t host_api_version);
#if RKG_ENABLE_D3D12
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_renderer_d3d12(uint32_t host_api_version);
#endif
#if RKG_ENABLE_SCRIPT_LUA
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_script_lua(uint32_t host_api_version);
#endif
#if RKG_ENABLE_SCRIPT_JS
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_script_js(uint32_t host_api_version);
#endif
#if RKG_ENABLE_SCRIPT_PYTHON
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_script_python(uint32_t host_api_version);
#endif

struct RuntimeContext {
  rkg::ecs::Registry registry;
  std::unordered_map<std::string, rkg::ecs::Entity> entities_by_name;
};

struct EventDispatch {
  rkg::PluginHost* host = nullptr;
  std::string* renderer_name = nullptr;
};

void sdl_event_callback(const void* event, void* user_data) {
  auto* dispatch = static_cast<EventDispatch*>(user_data);
  if (!dispatch || !event) return;
  const SDL_Event* sdl_event = static_cast<const SDL_Event*>(event);
  if (sdl_event->type == SDL_EVENT_WINDOW_RESIZED) {
    const int width = sdl_event->window.data1;
    const int height = sdl_event->window.data2;
    if (dispatch->host && dispatch->renderer_name) {
      if (auto* api = dispatch->host->find_by_name(*dispatch->renderer_name)) {
        if (api->on_window_resized) {
          api->on_window_resized(width, height);
        }
      }
    }
  }
}

#if RKG_ENABLE_DATA_YAML
void load_level(RuntimeContext& ctx, const fs::path& path) {
  if (!fs::exists(path)) {
    rkg::log::warn("level file missing");
    return;
  }
  YAML::Node level = YAML::LoadFile(path.string());
  if (!level["entities"]) return;
  for (const auto& entity : level["entities"]) {
    const auto name = entity["name"] ? entity["name"].as<std::string>() : "entity";
    rkg::ecs::Transform transform;
    if (entity["transform"]) {
      auto t = entity["transform"];
      if (t["position"]) {
        auto p = t["position"];
        transform.position[0] = p[0].as<float>();
        transform.position[1] = p[1].as<float>();
        transform.position[2] = p[2].as<float>();
      }
      if (t["rotation"]) {
        auto r = t["rotation"];
        transform.rotation[0] = r[0].as<float>();
        transform.rotation[1] = r[1].as<float>();
        transform.rotation[2] = r[2].as<float>();
      }
      if (t["scale"]) {
        auto s = t["scale"];
        transform.scale[0] = s[0].as<float>();
        transform.scale[1] = s[1].as<float>();
        transform.scale[2] = s[2].as<float>();
      }
    }
    const auto entity_id = ctx.registry.create_entity();
    ctx.registry.set_transform(entity_id, transform);
    ctx.entities_by_name[name] = entity_id;
    rkg::log::info(std::string("level entity spawned: ") + name);
  }
}
#endif

int main(int argc, char** argv) {
  const auto paths = rkg::resolve_paths(argc > 0 ? argv[0] : nullptr, std::nullopt, "demo_game");
  rkg::log::init("rkg_minimal_app", paths.root);
  rkg::log::install_crash_handlers();

  rkg::platform::Platform platform;
  if (!platform.init({1280, 720, "rkg_minimal_app"})) {
    rkg::log::error("platform init failed");
    return 1;
  }

  rkg::PluginHost host;
  host.register_plugin(rkg_plugin_get_api_renderer_null(rkg::kRkgPluginApiVersion));
#if RKG_ENABLE_D3D12
  host.register_plugin(rkg_plugin_get_api_renderer_d3d12(rkg::kRkgPluginApiVersion));
#endif
#if RKG_ENABLE_VULKAN
  host.register_plugin(rkg_plugin_get_api_renderer_vulkan(rkg::kRkgPluginApiVersion));
#endif
#if RKG_ENABLE_SCRIPT_LUA
  host.register_plugin(rkg_plugin_get_api_script_lua(rkg::kRkgPluginApiVersion));
#endif
#if RKG_ENABLE_SCRIPT_JS
  host.register_plugin(rkg_plugin_get_api_script_js(rkg::kRkgPluginApiVersion));
#endif
#if RKG_ENABLE_SCRIPT_PYTHON
  host.register_plugin(rkg_plugin_get_api_script_python(rkg::kRkgPluginApiVersion));
#endif

  rkg::EngineConfig config = rkg::load_engine_config("config/engine.yaml");

  std::string renderer_plugin = "renderer_null";
  if (config.renderer == "vulkan") renderer_plugin = "renderer_vulkan";
  if (config.renderer == "d3d12") {
#if RKG_ENABLE_D3D12
    renderer_plugin = "renderer_d3d12";
#else
    rkg::log::warn("renderer d3d12 requested but disabled");
#endif
  }

  RuntimeContext runtime;
  rkg::HostContext host_ctx;
  host_ctx.platform = &platform;
  host_ctx.registry = &runtime.registry;

  if (!host.init_plugin(renderer_plugin, &host_ctx)) {
    rkg::log::warn("renderer init failed; falling back to null");
    renderer_plugin = "renderer_null";
    host.init_plugin(renderer_plugin, &host_ctx);
  }
  rkg::log::info(std::string("renderer: ") + renderer_plugin);

  EventDispatch dispatch;
  dispatch.host = &host;
  dispatch.renderer_name = &renderer_plugin;
  platform.set_event_callback(&sdl_event_callback, &dispatch);

  std::vector<std::string> scripting_plugins;
  for (const auto& script : config.scripting) {
#if RKG_ENABLE_SCRIPT_LUA
    if (script == "lua") scripting_plugins.push_back("script_lua");
#endif
#if RKG_ENABLE_SCRIPT_JS
    if (script == "js") scripting_plugins.push_back("script_js");
#endif
#if RKG_ENABLE_SCRIPT_PYTHON
    if (script == "python") scripting_plugins.push_back("script_python");
#endif
  }
  for (const auto& name : scripting_plugins) {
    host.init_plugin(name, &host_ctx);
  }

#if RKG_ENABLE_DATA_YAML
  load_level(runtime, "content/levels/demo_level.yaml");
#endif
  rkg::InstructionContext inst_ctx;
  inst_ctx.registry = &runtime.registry;
  inst_ctx.entities_by_name = &runtime.entities_by_name;
  rkg::apply_runtime_instructions("build/runtime_instructions.json", inst_ctx);

  while (!platform.should_quit()) {
    platform.poll_events();
    const float dt = platform.delta_seconds();
    host.update_plugin(renderer_plugin, dt);
    for (const auto& name : scripting_plugins) {
      host.update_plugin(name, dt);
    }
  }

  for (const auto& name : scripting_plugins) {
    host.shutdown_plugin(name);
  }
  host.shutdown_plugin(renderer_plugin);
  platform.shutdown();
  rkg::log::shutdown();
  return 0;
}
