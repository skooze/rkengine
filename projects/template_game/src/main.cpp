#include "rkg/ecs.h"
#include "rkg/content_pack.h"
#include "rkg/host_context.h"
#include "rkg/instructions.h"
#include "rkg/log.h"
#include "rkg/paths.h"
#include "rkg/input.h"
#include "rkg/plugin_api.h"
#include "rkg/plugin_host.h"
#include "rkg/project.h"
#include "rkg/renderer_select.h"
#include "rkg_platform/platform.h"
#include "rkg_platform/file_watcher.h"

#include <SDL3/SDL.h>

#if RKG_ENABLE_IMGUI
#include "rkg_debug_ui/imgui_api.h"
#endif

#if RKG_ENABLE_DATA_YAML
#include <yaml-cpp/yaml.h>
#endif
#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
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
#if RKG_ENABLE_IMGUI
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_debug_ui_imgui(uint32_t host_api_version);
#endif

struct EventDispatch {
  rkg::PluginHost* host = nullptr;
  std::string* renderer_name = nullptr;
  bool debug_ui_enabled = false;
};

void sdl_event_callback(const void* event, void* user_data) {
  auto* dispatch = static_cast<EventDispatch*>(user_data);
  if (!dispatch || !event) return;
  const SDL_Event* sdl_event = static_cast<const SDL_Event*>(event);
#if RKG_ENABLE_IMGUI
  if (dispatch->debug_ui_enabled) {
    rkg::debug_ui::process_event(sdl_event);
  }
#endif
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

struct RuntimeContext {
  rkg::ecs::Registry registry;
  std::unordered_map<std::string, rkg::ecs::Entity> entities_by_name;
  rkg::ecs::Entity player = rkg::ecs::kInvalidEntity;
};

struct CookStatus {
  bool cook_ok = false;
  std::string last_success;
  std::string last_error;
  fs::path content_index_path;
  fs::path content_pack_path;
};

bool load_cook_status(const fs::path& path, CookStatus& out, std::string& error) {
#if !RKG_ENABLE_DATA_JSON
  (void)path;
  (void)out;
  error = "json disabled";
  return false;
#else
  if (!fs::exists(path)) {
    error = "cook_status missing";
    return false;
  }
  std::ifstream in(path);
  if (!in) {
    error = "cook_status open failed";
    return false;
  }
  nlohmann::json j;
  in >> j;
  out.cook_ok = j.value("cook_ok", false);
  out.last_success = j.value("last_success_timestamp", "");
  out.last_error = j.value("last_error", "");
  out.content_index_path = fs::path(j.value("content_index_path", ""));
  out.content_pack_path = fs::path(j.value("content_pack_path", ""));
  return true;
#endif
}

std::string renderer_caps_to_string(uint32_t caps) {
  std::string out;
  auto add = [&](const char* name) {
    if (!out.empty()) out += " | ";
    out += name;
  };
  if (caps & static_cast<uint32_t>(rkg::RendererCaps::DrawMesh)) add("DrawMesh");
  if (caps & static_cast<uint32_t>(rkg::RendererCaps::DrawPrimitive)) add("DrawPrimitive");
  if (caps & static_cast<uint32_t>(rkg::RendererCaps::DebugUiSupported)) add("DebugUiSupported");
  if (out.empty()) out = "None";
  return out;
}

std::string now_iso() {
  const auto now = std::chrono::system_clock::now();
  const auto tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &tt);
#else
  localtime_r(&tt, &tm);
#endif
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
  return std::string(buf);
}

uint64_t file_mtime_epoch(const fs::path& path) {
  std::error_code ec;
  auto ftime = fs::last_write_time(path, ec);
  if (ec) return 0;
  auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
      ftime - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::seconds>(sctp.time_since_epoch()).count());
}

fs::path normalize_content_path(const std::string& value) {
  fs::path p = value;
  if (p.is_absolute()) return p;
  std::string s = p.generic_string();
  const std::string prefix = "content/";
  if (s.rfind(prefix, 0) == 0) {
    s = s.substr(prefix.size());
  }
  return fs::path(s);
}

fs::path resolve_content_path(const fs::path& content_root, const std::string& value) {
  const fs::path rel = normalize_content_path(value);
  if (rel.is_absolute()) return rel;
  return content_root / rel;
}

#if RKG_ENABLE_DATA_JSON
void apply_transform_from_json(const nlohmann::json& t, rkg::ecs::Transform& transform) {
  if (t.contains("position") && t["position"].is_array()) {
    const auto& p = t["position"];
    if (p.size() > 0) transform.position[0] = p[0].get<float>();
    if (p.size() > 1) transform.position[1] = p[1].get<float>();
    if (p.size() > 2) transform.position[2] = p[2].get<float>();
  }
  if (t.contains("rotation") && t["rotation"].is_array()) {
    const auto& r = t["rotation"];
    if (r.size() > 0) transform.rotation[0] = r[0].get<float>();
    if (r.size() > 1) transform.rotation[1] = r[1].get<float>();
    if (r.size() > 2) transform.rotation[2] = r[2].get<float>();
  }
  if (t.contains("scale") && t["scale"].is_array()) {
    const auto& s = t["scale"];
    if (s.size() > 0) transform.scale[0] = s[0].get<float>();
    if (s.size() > 1) transform.scale[1] = s[1].get<float>();
    if (s.size() > 2) transform.scale[2] = s[2].get<float>();
  }
}

bool load_level_from_json(RuntimeContext& ctx, const nlohmann::json& level) {
  if (!level.contains("entities") || !level["entities"].is_array()) {
    return false;
  }
  for (const auto& entity : level["entities"]) {
    const auto name = entity.value("name", "entity");
    rkg::ecs::Transform transform;
    if (entity.contains("transform") && entity["transform"].is_object()) {
      apply_transform_from_json(entity["transform"], transform);
    }
    const auto entity_id = ctx.registry.create_entity();
    ctx.registry.set_transform(entity_id, transform);
    ctx.entities_by_name[name] = entity_id;
    if (ctx.player == rkg::ecs::kInvalidEntity) {
      ctx.player = entity_id;
    }
    rkg::log::info(std::string("level entity spawned: ") + name);
  }
  return true;
}
#endif

bool load_level_from_pack(RuntimeContext& ctx,
                          rkg::content::PackReader& pack_reader,
                          const std::string& level_path) {
#if RKG_ENABLE_DATA_JSON
  const std::string logical = normalize_content_path(level_path).generic_string();
  const auto* entry = pack_reader.find_by_path(logical);
  if (!entry) {
    rkg::log::warn("pack entry not found for level");
    return false;
  }
  std::string payload;
  if (!pack_reader.read_entry_data(*entry, payload)) {
    rkg::log::warn("pack entry read failed");
    return false;
  }
  const auto level = nlohmann::json::parse(payload, nullptr, false);
  if (level.is_discarded()) {
    rkg::log::warn("pack JSON parse failed");
    return false;
  }
  return load_level_from_json(ctx, level);
#else
  (void)ctx;
  (void)pack_reader;
  (void)level_path;
  rkg::log::warn("pack load requested but JSON disabled");
  return false;
#endif
}

#if RKG_ENABLE_DATA_YAML
void load_level_from_yaml(RuntimeContext& ctx, const fs::path& content_root, const std::string& level_path) {
  const fs::path path = resolve_content_path(content_root, level_path);
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
    if (ctx.player == rkg::ecs::kInvalidEntity) {
      ctx.player = entity_id;
    }
    rkg::log::info(std::string("level entity spawned: ") + name);
  }
}
#endif

void load_level(RuntimeContext& ctx,
                const fs::path& content_root,
                const std::string& level_path,
                rkg::content::PackReader* pack_reader) {
  if (pack_reader) {
    if (load_level_from_pack(ctx, *pack_reader, level_path)) {
      return;
    }
  }
#if RKG_ENABLE_DATA_YAML
  load_level_from_yaml(ctx, content_root, level_path);
#else
  (void)content_root;
  (void)level_path;
#endif
}

int main(int argc, char** argv) {
  const auto paths = rkg::resolve_paths(argc > 0 ? argv[0] : nullptr, std::nullopt, "template_game");
  rkg::log::init("rkg_game", paths.root);
  rkg::log::install_crash_handlers();
  const fs::path project_root = paths.project;
  const fs::path project_config_path = project_root / "project.yaml";
  rkg::ProjectConfig project = rkg::load_project_config(project_config_path);
#if RKG_ENABLE_IMGUI
  rkg::debug_ui::set_root_path(paths.root);
#endif
  rkg::input::InputMap input_map;
  if (!project.input_map.empty()) {
    input_map.load_from_file(project_root / project.input_map);
  }
  rkg::input::InputSystem input;
  input.set_map(input_map);

  const std::string project_name = !project.name.empty() ? project.name : project_root.filename().string();
  const fs::path raw_content_root = project_root / "content";
  const fs::path cooked_root = paths.content_cache_root / project_name;
  const fs::path cooked_index = cooked_root / "content.index.json";
  const fs::path cook_status_path = cooked_root / "cook_status.json";
  const fs::path pack_path = cooked_root / "content.pack";
  uint64_t cooked_mtime = file_mtime_epoch(cooked_index);
  bool cooked_available = cooked_mtime > 0;
  fs::path content_root = raw_content_root;
  bool using_cooked = false;
  rkg::content::PackReader pack_reader;
  bool pack_loaded = false;
  uint64_t pack_mtime = file_mtime_epoch(pack_path);
  uint64_t cook_status_mtime = file_mtime_epoch(cook_status_path);
  std::string last_cook_success;
  std::string last_cook_error;
  auto last_cook_check = std::chrono::steady_clock::time_point{};
  if (cooked_available) {
    content_root = cooked_root;
    using_cooked = true;
    rkg::log::info(std::string("content: using cooked cache at ") + cooked_root.string());
  } else if (project.dev_mode) {
    rkg::log::warn("cooked content missing; using raw content (dev_mode)");
  } else {
    rkg::log::error("cooked content missing and dev_mode is false");
    return 1;
  }

  if (using_cooked && pack_mtime > 0) {
    std::string pack_error;
    if (pack_reader.load(pack_path, pack_error)) {
      pack_loaded = true;
      rkg::log::info("content pack loaded");
    } else {
      rkg::log::warn(std::string("content pack load failed: ") + pack_error);
      pack_loaded = false;
    }
  }

  rkg::platform::Platform platform;
  if (!platform.init({1280, 720, "rkg_game"})) {
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
#if RKG_ENABLE_IMGUI
  host.register_plugin(rkg_plugin_get_api_debug_ui_imgui(rkg::kRkgPluginApiVersion));
#endif

  RuntimeContext runtime;
  rkg::HostContext host_ctx;
  host_ctx.platform = &platform;
  host_ctx.registry = &runtime.registry;

  std::string renderer_plugin;
  const auto order = rkg::build_renderer_fallback_order(project.renderer);
  const std::string requested_plugin = order.empty() ? "renderer_null" : order.front();
  for (const auto& candidate : order) {
    if (!host.find_by_name(candidate)) {
      if (candidate == requested_plugin) {
        rkg::log::warn("requested renderer not available: " + candidate);
      }
      continue;
    }
    if (host.init_plugin(candidate, &host_ctx)) {
      renderer_plugin = candidate;
      if (candidate != requested_plugin) {
        rkg::log::warn("renderer fallback to " + candidate);
      }
      break;
    }
    rkg::log::warn("renderer init failed: " + candidate);
  }
  if (renderer_plugin.empty()) {
    rkg::log::error("no renderer available");
    return 1;
  }
  rkg::log::info(std::string("renderer: ") + renderer_plugin);

  uint32_t renderer_caps = 0;
  if (auto* api = host.find_by_name(renderer_plugin)) {
    renderer_caps = api->caps;
    rkg::log::info(std::string("renderer caps: ") + renderer_caps_to_string(renderer_caps));
    const bool can_draw = (renderer_caps & static_cast<uint32_t>(rkg::RendererCaps::DrawPrimitive)) ||
                          (renderer_caps & static_cast<uint32_t>(rkg::RendererCaps::DrawMesh));
    if (!can_draw) {
      rkg::log::warn("renderer lacks draw capability; demo may be limited");
    }
  }

  std::vector<std::string> active_plugins = project.plugins;
#if RKG_ENABLE_IMGUI
  bool debug_ui_enabled = false;
#endif
  for (const auto& name : active_plugins) {
    if (name == renderer_plugin) continue;
#if RKG_ENABLE_IMGUI
    if (name == "debug_ui_imgui" &&
        (renderer_caps & static_cast<uint32_t>(rkg::RendererCaps::DebugUiSupported)) == 0) {
      rkg::log::warn("debug_ui disabled: renderer does not support DebugUi");
      continue;
    }
#endif
    if (host.init_plugin(name, &host_ctx)) {
#if RKG_ENABLE_IMGUI
      if (name == "debug_ui_imgui") {
        debug_ui_enabled = true;
        rkg::log::info("debug_ui enabled");
      }
#endif
    }
  }

  EventDispatch dispatch;
  dispatch.host = &host;
  dispatch.renderer_name = &renderer_plugin;
#if RKG_ENABLE_IMGUI
  dispatch.debug_ui_enabled = debug_ui_enabled;
#endif
  platform.set_event_callback(&sdl_event_callback, &dispatch);

  if (!project.initial_level.empty()) {
    rkg::content::PackReader* pack_ptr = (using_cooked && pack_loaded) ? &pack_reader : nullptr;
    load_level(runtime, content_root, project.initial_level, pack_ptr);
  }

  rkg::platform::FileWatcher watcher;
  watcher.start(raw_content_root);
  rkg::log::info(std::string("content watcher: ") + watcher.backend_name());

  rkg::InstructionContext inst_ctx;
  inst_ctx.registry = &runtime.registry;
  inst_ctx.entities_by_name = &runtime.entities_by_name;
  rkg::apply_runtime_instructions(paths.root / "build/runtime_instructions.json", inst_ctx);

  while (!platform.should_quit()) {
    platform.poll_events();
    const float dt = platform.delta_seconds();
    input.update(platform);

    if (input.action("Quit").pressed) {
      platform.request_quit();
    }
#if RKG_ENABLE_IMGUI
    if (input.action("ToggleDebugUI").pressed) {
      rkg::debug_ui::set_visible(!rkg::debug_ui::is_visible());
    }
#endif
    host.update_plugin(renderer_plugin, dt);
    for (const auto& name : active_plugins) {
      if (name == renderer_plugin) continue;
      host.update_plugin(name, dt);
    }

    bool auto_reload_requested = false;
    if (project.dev_mode) {
      const auto now = std::chrono::steady_clock::now();
      if (last_cook_check == std::chrono::steady_clock::time_point() ||
          std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cook_check).count() > 200) {
        last_cook_check = now;
        const uint64_t status_mtime = file_mtime_epoch(cook_status_path);
        if (status_mtime > 0 && status_mtime != cook_status_mtime) {
          cook_status_mtime = status_mtime;
          CookStatus status;
          std::string error;
          if (load_cook_status(cook_status_path, status, error)) {
            if (status.cook_ok) {
              if (!status.last_success.empty() && status.last_success != last_cook_success) {
                last_cook_success = status.last_success;
              }
              auto_reload_requested = true;
            } else if (!status.last_error.empty() && status.last_error != last_cook_error) {
              last_cook_error = status.last_error;
              rkg::log::warn(std::string("cook_status: ") + status.last_error);
            }
          } else if (!error.empty() && error != last_cook_error) {
            last_cook_error = error;
            rkg::log::warn(std::string("cook_status read failed: ") + error);
          }
        }
      }
    }

    std::vector<rkg::platform::FileChange> changes;
    watcher.poll(changes);
    const bool raw_changed = !changes.empty();
    bool reload_requested = raw_changed || input.action("Reload").pressed || auto_reload_requested;
#if RKG_ENABLE_IMGUI
    reload_requested = reload_requested || rkg::debug_ui::consume_force_reload_request();
#endif
    if (reload_requested) {
      const uint64_t cooked_before = cooked_mtime;
      const uint64_t cooked_now = file_mtime_epoch(cooked_index);
      if (cooked_now == 0) {
        cooked_available = false;
      } else if (cooked_now != cooked_mtime) {
        cooked_mtime = cooked_now;
        cooked_available = true;
        content_root = cooked_root;
        using_cooked = true;
      }

      std::string reload_error;
      if (raw_changed && using_cooked && cooked_now <= cooked_before) {
        reload_error = "raw content changed; run rkgctl content cook";
      }

      bool can_reload = true;
      if (!cooked_available) {
        if (project.dev_mode) {
          content_root = raw_content_root;
          using_cooked = false;
          if (reload_error.empty()) {
            reload_error = "cooked cache missing; using raw content";
          }
        } else {
          rkg::log::error("cooked content missing and dev_mode is false");
          reload_error = "cooked cache missing; reload skipped";
          can_reload = false;
        }
      }

      if (can_reload) {
        const uint64_t pack_now = file_mtime_epoch(pack_path);
        if (using_cooked && pack_now > 0 && pack_now != pack_mtime) {
          pack_mtime = pack_now;
          std::string pack_error;
          if (pack_reader.load(pack_path, pack_error)) {
            pack_loaded = true;
            rkg::log::info("content pack reloaded");
          } else {
            pack_loaded = false;
            rkg::log::warn(std::string("content pack reload failed: ") + pack_error);
          }
        } else if (pack_now == 0) {
          pack_mtime = 0;
          pack_loaded = false;
        }

        runtime.registry = rkg::ecs::Registry{};
        runtime.entities_by_name.clear();
        runtime.player = rkg::ecs::kInvalidEntity;
        if (!project.initial_level.empty()) {
          rkg::content::PackReader* pack_ptr = (using_cooked && pack_loaded) ? &pack_reader : nullptr;
          load_level(runtime, content_root, project.initial_level, pack_ptr);
        }
        rkg::log::info(std::string("content reloaded (") + (using_cooked ? "cooked" : "raw") + ")");
      }
#if RKG_ENABLE_IMGUI
      rkg::debug_ui::set_hot_reload_status(now_iso(), reload_error);
#endif
    }

    if (runtime.player != rkg::ecs::kInvalidEntity) {
      if (auto* transform = runtime.registry.get_transform(runtime.player)) {
        const float speed = 2.0f;
        if (input.action("MoveForward").held) transform->position[1] += speed * dt;
        if (input.action("MoveBack").held) transform->position[1] -= speed * dt;
        if (input.action("MoveLeft").held) transform->position[0] -= speed * dt;
        if (input.action("MoveRight").held) transform->position[0] += speed * dt;
      }
    }
  }

  for (const auto& name : active_plugins) {
    if (name == renderer_plugin) continue;
    host.shutdown_plugin(name);
  }
  host.shutdown_plugin(renderer_plugin);
  platform.shutdown();
  rkg::log::shutdown();
  return 0;
}
