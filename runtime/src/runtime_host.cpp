#include "rkg/runtime_host.h"

#include "rkg/content_pack.h"
#include "rkg/instructions.h"
#include "rkg/log.h"
#include "rkg/plugin_api.h"
#include "rkg/renderer_select.h"
#include "rkg/renderer_util.h"

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

#include <algorithm>
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

namespace rkg::runtime {
namespace {
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

bool parse_mesh_id(const std::string& value, rkg::ecs::MeshId& out) {
  if (value == "cube") {
    out = rkg::ecs::MeshId::Cube;
    return true;
  }
  if (value == "quad") {
    out = rkg::ecs::MeshId::Quad;
    return true;
  }
  out = rkg::ecs::MeshId::Unknown;
  return false;
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

bool apply_renderable_from_json(const nlohmann::json& node, rkg::ecs::Renderable& renderable) {
  bool touched = true;
  if (node.contains("mesh")) {
    if (node["mesh"].is_string()) {
      const auto mesh = node["mesh"].get<std::string>();
      if (!parse_mesh_id(mesh, renderable.mesh)) {
        rkg::log::warn(std::string("renderable mesh unknown: ") + mesh);
      }
    } else {
      rkg::log::warn("renderable.mesh must be a string");
    }
  }
  if (node.contains("color")) {
    if (node["color"].is_array()) {
      const auto& c = node["color"];
      if (c.size() >= 3) {
        renderable.color[0] = c[0].get<float>();
        renderable.color[1] = c[1].get<float>();
        renderable.color[2] = c[2].get<float>();
        renderable.color[3] = (c.size() > 3) ? c[3].get<float>() : 1.0f;
      } else {
        rkg::log::warn("renderable.color must have 3 or 4 values");
      }
    } else {
      rkg::log::warn("renderable.color must be an array");
    }
  }
  return touched;
}

bool apply_prefab_components_json(const nlohmann::json& prefab,
                                  rkg::ecs::Transform& transform,
                                  rkg::ecs::Renderable& renderable,
                                  bool& has_renderable) {
  if (!prefab.contains("components") || !prefab["components"].is_object()) {
    return false;
  }
  const auto& comps = prefab["components"];
  if (comps.contains("Transform") && comps["Transform"].is_object()) {
    apply_transform_from_json(comps["Transform"], transform);
  }
  if (comps.contains("Renderable") && comps["Renderable"].is_object()) {
    has_renderable = apply_renderable_from_json(comps["Renderable"], renderable) || has_renderable;
  }
  return true;
}

bool load_level_from_json(rkg::ecs::Registry& registry,
                          std::unordered_map<std::string, rkg::ecs::Entity>& entities_by_name,
                          std::unordered_map<rkg::ecs::Entity, std::string>& entity_override_keys,
                          std::unordered_map<std::string, rkg::ecs::Entity>& entities_by_override_key,
                          rkg::ecs::Entity& player,
                          const nlohmann::json& level,
                          rkg::content::PackReader* pack_reader) {
  if (!level.contains("entities") || !level["entities"].is_array()) {
    return false;
  }
  for (const auto& entity : level["entities"]) {
    const auto name = entity.value("name", "entity");
    std::string override_key = name;
    if (entity.contains("id") && entity["id"].is_string()) {
      override_key = entity["id"].get<std::string>();
    }
    rkg::ecs::Transform transform;
    rkg::ecs::Renderable renderable;
    bool has_renderable = false;
    std::string prefab_ref;
    if (entity.contains("prefab") && entity["prefab"].is_string()) {
      prefab_ref = entity["prefab"].get<std::string>();
    } else if (entity.contains("prefab_ref") && entity["prefab_ref"].is_string()) {
      prefab_ref = entity["prefab_ref"].get<std::string>();
    }
    if (!prefab_ref.empty() && pack_reader) {
      const fs::path base = normalize_content_path(prefab_ref);
      std::vector<fs::path> candidates;
      const std::string base_str = base.generic_string();
      const bool has_slash = base_str.find('/') != std::string::npos;
      const bool has_ext = base.has_extension();
      if (has_slash || has_ext) {
        candidates.push_back(base);
        if (!has_slash) {
          candidates.push_back(fs::path("prefabs") / base);
        }
      } else {
        candidates.push_back(fs::path("prefabs") / (prefab_ref + ".yaml"));
        candidates.push_back(fs::path("prefabs") / (prefab_ref + ".yml"));
        candidates.push_back(fs::path("prefabs") / (prefab_ref + ".json"));
      }

      bool prefab_loaded = false;
      for (const auto& candidate : candidates) {
        const auto* entry = pack_reader->find_by_path(candidate.generic_string());
        if (!entry) continue;
        std::string payload;
        if (!pack_reader->read_entry_data(*entry, payload)) {
          continue;
        }
        const auto prefab = nlohmann::json::parse(payload, nullptr, false);
        if (prefab.is_discarded()) {
          continue;
        }
        apply_prefab_components_json(prefab, transform, renderable, has_renderable);
        prefab_loaded = true;
        break;
      }
      if (!prefab_loaded) {
        rkg::log::warn(std::string("prefab not found in pack: ") + prefab_ref);
      }
    }

    if (entity.contains("transform") && entity["transform"].is_object()) {
      apply_transform_from_json(entity["transform"], transform);
    }
    if (entity.contains("renderable") && entity["renderable"].is_object()) {
      has_renderable = apply_renderable_from_json(entity["renderable"], renderable) || has_renderable;
    }
    const auto entity_id = registry.create_entity();
    registry.set_transform(entity_id, transform);
    if (has_renderable) {
      registry.set_renderable(entity_id, renderable);
    }
    entities_by_name[name] = entity_id;
    entity_override_keys[entity_id] = override_key;
    const auto inserted = entities_by_override_key.emplace(override_key, entity_id);
    if (!inserted.second) {
      rkg::log::warn(std::string("duplicate override id: ") + override_key);
      entities_by_override_key[override_key] = entity_id;
    }
    if (player == rkg::ecs::kInvalidEntity) {
      player = entity_id;
    }
    rkg::log::info(std::string("level entity spawned: ") + name);
  }
  return true;
}
#endif

bool load_level_from_pack(rkg::ecs::Registry& registry,
                          std::unordered_map<std::string, rkg::ecs::Entity>& entities_by_name,
                          std::unordered_map<rkg::ecs::Entity, std::string>& entity_override_keys,
                          std::unordered_map<std::string, rkg::ecs::Entity>& entities_by_override_key,
                          rkg::ecs::Entity& player,
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
  return load_level_from_json(registry, entities_by_name, entity_override_keys, entities_by_override_key,
                              player, level, &pack_reader);
#else
  (void)registry;
  (void)entities_by_name;
  (void)player;
  (void)pack_reader;
  (void)level_path;
  rkg::log::warn("pack load requested but JSON disabled");
  return false;
#endif
}

#if RKG_ENABLE_DATA_YAML
bool apply_renderable_from_yaml(const YAML::Node& node, rkg::ecs::Renderable& renderable) {
  bool touched = true;
  if (node["mesh"]) {
    if (node["mesh"].IsScalar()) {
      const auto mesh = node["mesh"].as<std::string>();
      if (!parse_mesh_id(mesh, renderable.mesh)) {
        rkg::log::warn(std::string("renderable mesh unknown: ") + mesh);
      }
    } else {
      rkg::log::warn("renderable.mesh must be a string");
    }
  }
  if (node["color"]) {
    if (node["color"].IsSequence()) {
      const auto c = node["color"];
      if (c.size() >= 3) {
        renderable.color[0] = c[0].as<float>();
        renderable.color[1] = c[1].as<float>();
        renderable.color[2] = c[2].as<float>();
        renderable.color[3] = (c.size() > 3) ? c[3].as<float>() : 1.0f;
      } else {
        rkg::log::warn("renderable.color must have 3 or 4 values");
      }
    } else {
      rkg::log::warn("renderable.color must be a list");
    }
  }
  return touched;
}

bool apply_prefab_components_yaml(const YAML::Node& prefab,
                                  rkg::ecs::Transform& transform,
                                  rkg::ecs::Renderable& renderable,
                                  bool& has_renderable) {
  if (!prefab["components"] || !prefab["components"].IsMap()) {
    return false;
  }
  const auto comps = prefab["components"];
  if (comps["Transform"] && comps["Transform"].IsMap()) {
    const auto t = comps["Transform"];
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
  if (comps["Renderable"] && comps["Renderable"].IsMap()) {
    has_renderable = apply_renderable_from_yaml(comps["Renderable"], renderable) || has_renderable;
  }
  return true;
}

void load_level_from_yaml(rkg::ecs::Registry& registry,
                          std::unordered_map<std::string, rkg::ecs::Entity>& entities_by_name,
                          std::unordered_map<rkg::ecs::Entity, std::string>& entity_override_keys,
                          std::unordered_map<std::string, rkg::ecs::Entity>& entities_by_override_key,
                          rkg::ecs::Entity& player,
                          const fs::path& content_root,
                          const std::string& level_path) {
  const fs::path path = resolve_content_path(content_root, level_path);
  if (!fs::exists(path)) {
    rkg::log::warn("level file missing");
    return;
  }
  YAML::Node level = YAML::LoadFile(path.string());
  if (!level["entities"]) return;
  for (const auto& entity : level["entities"]) {
    const auto name = entity["name"] ? entity["name"].as<std::string>() : "entity";
    std::string override_key = name;
    if (entity["id"] && entity["id"].IsScalar()) {
      override_key = entity["id"].as<std::string>();
    }
    rkg::ecs::Transform transform;
    rkg::ecs::Renderable renderable;
    bool has_renderable = false;
    std::string prefab_ref;
    if (entity["prefab"] && entity["prefab"].IsScalar()) {
      prefab_ref = entity["prefab"].as<std::string>();
    } else if (entity["prefab_ref"] && entity["prefab_ref"].IsScalar()) {
      prefab_ref = entity["prefab_ref"].as<std::string>();
    }
    if (!prefab_ref.empty()) {
      fs::path base = normalize_content_path(prefab_ref);
      std::vector<fs::path> candidates;
      const std::string base_str = base.generic_string();
      const bool has_slash = base_str.find('/') != std::string::npos;
      const bool has_ext = base.has_extension();
      if (has_slash || has_ext) {
        candidates.push_back(base);
        if (!has_slash) {
          candidates.push_back(fs::path("prefabs") / base);
        }
      } else {
        candidates.push_back(fs::path("prefabs") / (prefab_ref + ".yaml"));
        candidates.push_back(fs::path("prefabs") / (prefab_ref + ".yml"));
        candidates.push_back(fs::path("prefabs") / (prefab_ref + ".json"));
      }

      bool prefab_loaded = false;
      for (const auto& candidate : candidates) {
        const fs::path path = resolve_content_path(content_root, candidate.generic_string());
        if (!fs::exists(path)) continue;
        try {
          YAML::Node prefab = YAML::LoadFile(path.string());
          apply_prefab_components_yaml(prefab, transform, renderable, has_renderable);
          prefab_loaded = true;
          break;
        } catch (const std::exception& e) {
          rkg::log::warn(std::string("prefab YAML load failed: ") + e.what());
        }
      }
      if (!prefab_loaded) {
        rkg::log::warn(std::string("prefab not found: ") + prefab_ref);
      }
    }
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
    if (entity["renderable"] && entity["renderable"].IsMap()) {
      has_renderable = apply_renderable_from_yaml(entity["renderable"], renderable) || has_renderable;
    }
    const auto entity_id = registry.create_entity();
    registry.set_transform(entity_id, transform);
    if (has_renderable) {
      registry.set_renderable(entity_id, renderable);
    }
    entities_by_name[name] = entity_id;
    entity_override_keys[entity_id] = override_key;
    const auto inserted = entities_by_override_key.emplace(override_key, entity_id);
    if (!inserted.second) {
      rkg::log::warn(std::string("duplicate override id: ") + override_key);
      entities_by_override_key[override_key] = entity_id;
    }
    if (player == rkg::ecs::kInvalidEntity) {
      player = entity_id;
    }
    rkg::log::info(std::string("level entity spawned: ") + name);
  }
}
#endif

void load_level(rkg::ecs::Registry& registry,
                std::unordered_map<std::string, rkg::ecs::Entity>& entities_by_name,
                std::unordered_map<rkg::ecs::Entity, std::string>& entity_override_keys,
                std::unordered_map<std::string, rkg::ecs::Entity>& entities_by_override_key,
                rkg::ecs::Entity& player,
                const fs::path& content_root,
                const std::string& level_path,
                rkg::content::PackReader* pack_reader) {
  if (pack_reader) {
    if (load_level_from_pack(registry, entities_by_name, entity_override_keys, entities_by_override_key,
                             player, *pack_reader, level_path)) {
      return;
    }
  }
#if RKG_ENABLE_DATA_YAML
  load_level_from_yaml(registry, entities_by_name, entity_override_keys, entities_by_override_key,
                       player, content_root, level_path);
#else
  (void)registry;
  (void)entities_by_name;
  (void)entity_override_keys;
  (void)entities_by_override_key;
  (void)player;
  (void)content_root;
  (void)level_path;
#endif
}

void apply_editor_overrides(rkg::ecs::Registry& registry,
                            const std::unordered_map<std::string, rkg::ecs::Entity>& entities_by_override_key,
                            const fs::path& overrides_path) {
#if RKG_ENABLE_DATA_YAML
  if (!fs::exists(overrides_path)) {
    return;
  }
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(overrides_path.string());
  } catch (const std::exception& e) {
    rkg::log::warn(std::string("editor_overrides load failed: ") + e.what());
    return;
  }
  if (!doc["overrides"] || !doc["overrides"].IsMap()) {
    return;
  }
  const auto overrides = doc["overrides"];
  for (const auto& item : overrides) {
    const auto key = item.first.as<std::string>();
    const auto it = entities_by_override_key.find(key);
    if (it == entities_by_override_key.end()) {
      rkg::log::warn(std::string("override target missing: ") + key);
      continue;
    }
    const auto entity = it->second;
    const auto node = item.second;
    if (node["transform"] && node["transform"].IsMap()) {
      auto* transform = registry.get_transform(entity);
      if (transform) {
        const auto t = node["transform"];
        if (t["position"]) {
          auto p = t["position"];
          transform->position[0] = p[0].as<float>();
          transform->position[1] = p[1].as<float>();
          transform->position[2] = p[2].as<float>();
        }
        if (t["rotation"]) {
          auto r = t["rotation"];
          transform->rotation[0] = r[0].as<float>();
          transform->rotation[1] = r[1].as<float>();
          transform->rotation[2] = r[2].as<float>();
        }
        if (t["scale"]) {
          auto s = t["scale"];
          transform->scale[0] = s[0].as<float>();
          transform->scale[1] = s[1].as<float>();
          transform->scale[2] = s[2].as<float>();
        }
      }
    }
    if (node["renderable"] && node["renderable"].IsMap()) {
      const auto rnode = node["renderable"];
      rkg::ecs::Renderable renderable{};
      bool had = false;
      if (auto* existing = registry.get_renderable(entity)) {
        renderable = *existing;
        had = true;
      }
      if (apply_renderable_from_yaml(rnode, renderable)) {
        registry.set_renderable(entity, renderable);
      } else if (had) {
        registry.set_renderable(entity, renderable);
      }
    }
  }
#else
  (void)registry;
  (void)entities_by_override_key;
  (void)overrides_path;
#endif
}

fs::path detect_executable_dir(const char* argv0) {
#if defined(_WIN32)
  if (argv0) {
    return fs::absolute(argv0).parent_path();
  }
  return fs::current_path();
#else
  if (argv0) {
    return fs::absolute(argv0).parent_path();
  }
  return fs::current_path();
#endif
}

} // namespace

void RuntimeHost::sdl_event_callback(const void* event, void* user_data) {
  auto* self = static_cast<RuntimeHost*>(user_data);
  if (!self || !event) return;
  const SDL_Event* sdl_event = static_cast<const SDL_Event*>(event);
#if RKG_ENABLE_IMGUI
  if (self->debug_ui_enabled_) {
    rkg::debug_ui::process_event(sdl_event);
  }
#endif
#if RKG_DEBUG
  static int logged_mouse_events = 0;
  if (logged_mouse_events < 2) {
    if (sdl_event->type == SDL_EVENT_MOUSE_MOTION ||
        sdl_event->type == SDL_EVENT_MOUSE_BUTTON_DOWN ||
        sdl_event->type == SDL_EVENT_MOUSE_BUTTON_UP) {
      const std::string msg = std::string("event: mouse type=") +
                              std::to_string(sdl_event->type) +
                              " x=" + std::to_string(static_cast<int>(sdl_event->motion.x)) +
                              " y=" + std::to_string(static_cast<int>(sdl_event->motion.y));
      rkg::log::info(msg);
      logged_mouse_events++;
    }
  }
#endif
  if (sdl_event->type == SDL_EVENT_WINDOW_RESIZED ||
      sdl_event->type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
    const int width = sdl_event->window.data1;
    const int height = sdl_event->window.data2;
    if (!self->renderer_plugin_.empty()) {
      if (auto* api = self->host_.find_by_name(self->renderer_plugin_)) {
        if (api->on_window_resized) {
          api->on_window_resized(width, height);
        }
      }
    }
  }
}

bool RuntimeHost::init(const RuntimeHostInit& init, std::string& error) {
  paths_ = rkg::resolve_paths(init.argv0, init.project_override, init.default_project);
  executable_dir_ = detect_executable_dir(init.argv0);

  rkg::log::init(init.app_name, paths_.root);
  rkg::log::install_crash_handlers();

  project_root_ = paths_.project;
  const fs::path project_config_path = project_root_ / "project.yaml";
  project_ = rkg::load_project_config(project_config_path);
#if RKG_ENABLE_IMGUI
  rkg::debug_ui::set_root_path(paths_.root);
#endif

  project_name_ = !project_.name.empty() ? project_.name : project_root_.filename().string();

  if (!project_.input_map.empty()) {
    input_map_.load_from_file(project_root_ / project_.input_map);
  }
  input_.set_map(input_map_);

  raw_content_root_ = project_root_ / "content";
  cooked_root_ = paths_.content_cache_root / project_name_;
  cooked_index_ = cooked_root_ / "content.index.json";
  cook_status_path_ = cooked_root_ / "cook_status.json";
  pack_path_ = cooked_root_ / "content.pack";

  cooked_mtime_ = file_mtime_epoch(cooked_index_);
  cooked_available_ = cooked_mtime_ > 0;

  if (cooked_available_) {
    using_cooked_ = true;
    rkg::log::info(std::string("content: using cooked cache at ") + cooked_root_.string());
  } else if (project_.dev_mode) {
    rkg::log::warn("cooked content missing; using raw content (dev_mode)");
  } else {
    error = "cooked content missing and dev_mode is false";
    rkg::log::error(error);
    return false;
  }

  pack_mtime_ = file_mtime_epoch(pack_path_);
  if (using_cooked_ && pack_mtime_ > 0) {
    std::string pack_error;
    if (pack_reader_.load(pack_path_, pack_error)) {
      pack_loaded_ = true;
      rkg::log::info("content pack loaded");
    } else {
      rkg::log::warn(std::string("content pack load failed: ") + pack_error);
      pack_loaded_ = false;
    }
  }

  if (!platform_.init(init.window)) {
    error = "platform init failed";
    rkg::log::error(error);
    return false;
  }

  setup_plugins(init.force_debug_ui, init.disable_debug_ui, error);
  if (!error.empty()) {
    return false;
  }

  platform_.set_event_callback(&RuntimeHost::sdl_event_callback, this);

#if RKG_ENABLE_IMGUI
  if (debug_ui_enabled_) {
    rkg::debug_ui::set_renderer_name(rkg::renderer_display_name(renderer_plugin_));
  } else if (debug_ui_requested_) {
    debug_ui_unavailable_reason_ = std::string("Debug UI unavailable for ") +
                                   rkg::renderer_display_name(renderer_plugin_) +
                                   " (Vulkan-only).";
    const auto title = init.app_name + std::string(" (") + debug_ui_unavailable_reason_ + ")";
    if (auto* window = static_cast<SDL_Window*>(platform_.native_window())) {
      SDL_SetWindowTitle(window, title.c_str());
    }
    rkg::log::warn(debug_ui_unavailable_reason_);
  }
#endif

  load_initial_level();

  watcher_.start(raw_content_root_);
  rkg::log::info(std::string("content watcher: ") + watcher_.backend_name());

  rkg::InstructionContext inst_ctx;
  inst_ctx.registry = &registry_;
  inst_ctx.entities_by_name = &entities_by_name_;
  rkg::apply_runtime_instructions(paths_.root / "build/runtime_instructions.json", inst_ctx);

  return true;
}

void RuntimeHost::shutdown() {
  for (const auto& name : active_plugins_) {
    if (name == renderer_plugin_) continue;
    host_.shutdown_plugin(name);
  }
  if (!renderer_plugin_.empty()) {
    host_.shutdown_plugin(renderer_plugin_);
  }
  platform_.shutdown();
  rkg::log::shutdown();
}

void RuntimeHost::setup_plugins(bool force_debug_ui, bool disable_debug_ui, std::string& error) {
  host_.register_plugin(rkg_plugin_get_api_renderer_null(rkg::kRkgPluginApiVersion));
#if RKG_ENABLE_D3D12
  host_.register_plugin(rkg_plugin_get_api_renderer_d3d12(rkg::kRkgPluginApiVersion));
#endif
#if RKG_ENABLE_VULKAN
  host_.register_plugin(rkg_plugin_get_api_renderer_vulkan(rkg::kRkgPluginApiVersion));
#endif
#if RKG_ENABLE_SCRIPT_LUA
  host_.register_plugin(rkg_plugin_get_api_script_lua(rkg::kRkgPluginApiVersion));
#endif
#if RKG_ENABLE_SCRIPT_JS
  host_.register_plugin(rkg_plugin_get_api_script_js(rkg::kRkgPluginApiVersion));
#endif
#if RKG_ENABLE_SCRIPT_PYTHON
  host_.register_plugin(rkg_plugin_get_api_script_python(rkg::kRkgPluginApiVersion));
#endif
#if RKG_ENABLE_IMGUI
  host_.register_plugin(rkg_plugin_get_api_debug_ui_imgui(rkg::kRkgPluginApiVersion));
#endif

  host_ctx_.platform = &platform_;
  host_ctx_.registry = &registry_;

  const auto order = rkg::build_renderer_fallback_order(project_.renderer);
  const std::string requested_plugin = order.empty() ? "renderer_null" : order.front();
  for (const auto& candidate : order) {
    if (!host_.find_by_name(candidate)) {
      if (candidate == requested_plugin) {
        rkg::log::warn("requested renderer not available: " + candidate);
      }
      continue;
    }
    if (host_.init_plugin(candidate, &host_ctx_)) {
      renderer_plugin_ = candidate;
      if (candidate != requested_plugin) {
        rkg::log::warn("renderer fallback to " + candidate);
      }
      break;
    }
    rkg::log::warn("renderer init failed: " + candidate);
  }

  if (renderer_plugin_.empty()) {
    error = "no renderer available";
    rkg::log::error(error);
    return;
  }
  rkg::log::info(std::string("renderer: ") + renderer_plugin_);

  if (auto* api = host_.find_by_name(renderer_plugin_)) {
    renderer_caps_ = api->caps;
    rkg::log::info(std::string("renderer caps: ") + renderer_caps_to_string(renderer_caps_));
    const bool can_draw = (renderer_caps_ & static_cast<uint32_t>(rkg::RendererCaps::DrawPrimitive)) ||
                          (renderer_caps_ & static_cast<uint32_t>(rkg::RendererCaps::DrawMesh));
    if (!can_draw) {
      rkg::log::warn("renderer lacks draw capability; demo may be limited");
    }
  }

  active_plugins_ = project_.plugins;
#if RKG_ENABLE_IMGUI
  if (disable_debug_ui) {
    active_plugins_.erase(std::remove(active_plugins_.begin(), active_plugins_.end(), "debug_ui_imgui"),
                          active_plugins_.end());
    debug_ui_requested_ = false;
  } else {
    debug_ui_requested_ = std::find(active_plugins_.begin(), active_plugins_.end(), "debug_ui_imgui") !=
                          active_plugins_.end();
    if (force_debug_ui && !debug_ui_requested_) {
      active_plugins_.push_back("debug_ui_imgui");
      debug_ui_requested_ = true;
    }
  }
#endif

  for (const auto& name : active_plugins_) {
    if (name == renderer_plugin_) continue;
#if RKG_ENABLE_IMGUI
    if (name == "debug_ui_imgui" &&
        (renderer_caps_ & static_cast<uint32_t>(rkg::RendererCaps::DebugUiSupported)) == 0) {
      rkg::log::warn("debug_ui disabled: renderer does not support DebugUi");
      continue;
    }
#endif
    if (host_.init_plugin(name, &host_ctx_)) {
#if RKG_ENABLE_IMGUI
      if (name == "debug_ui_imgui") {
        debug_ui_enabled_ = true;
        rkg::log::info("debug_ui enabled");
      }
#endif
    }
  }
}

void RuntimeHost::update_input() {
  input_.update(platform_);
}

rkg::input::ActionState RuntimeHost::input_action(const std::string& name) const {
  return input_.action(name);
}

void RuntimeHost::tick(const FrameParams& params, const ActionStateProvider& action_state_provider) {
  const float frame_dt = params.frame_dt;
  const float sim_dt = params.run_simulation ? params.sim_dt : 0.0f;
  static bool logged_tick = false;
  const bool first_tick = !logged_tick;
  if (first_tick) {
    rkg::log::info("tick: begin");
    logged_tick = true;
  }

  if (params.update_input) {
    input_.update(platform_);
  }

  auto action_state = [&](const std::string& name) -> rkg::input::ActionState {
    if (action_state_provider) {
      return action_state_provider(name);
    }
    return input_.action(name);
  };

  if (action_state("Quit").pressed) {
    platform_.request_quit();
  }
#if RKG_ENABLE_IMGUI
  if (action_state("ToggleDebugUI").pressed) {
    rkg::debug_ui::set_visible(!rkg::debug_ui::is_visible());
  }
#endif

  const std::string debug_ui_plugin = "debug_ui_imgui";
  for (const auto& name : active_plugins_) {
    if (name == renderer_plugin_) continue;
    if (name == debug_ui_plugin) continue;
    float dt = frame_dt;
    if (auto* api = host_.find_by_name(name)) {
      if (api->type != rkg::PluginType::Renderer && api->type != rkg::PluginType::DebugUI) {
        dt = sim_dt;
      }
    }
    host_.update_plugin(name, dt);
  }
  if (!renderer_plugin_.empty()) {
    if (first_tick) {
      rkg::log::info("tick: renderer update");
    }
    host_.update_plugin(renderer_plugin_, frame_dt);
  }
  if (debug_ui_enabled_ && !debug_ui_plugin.empty()) {
    host_.update_plugin(debug_ui_plugin, frame_dt);
  }

  const bool manual_reload = manual_reload_requested_ || action_state("Reload").pressed;
  handle_reload(manual_reload);
  manual_reload_requested_ = false;
  manual_reload_reason_.clear();

  if (params.run_simulation && player_ != rkg::ecs::kInvalidEntity) {
    if (auto* transform = registry_.get_transform(player_)) {
      const float speed = 2.0f;
      if (action_state("MoveForward").held) transform->position[2] += speed * sim_dt;
      if (action_state("MoveBack").held) transform->position[2] -= speed * sim_dt;
      if (action_state("MoveLeft").held) transform->position[0] -= speed * sim_dt;
      if (action_state("MoveRight").held) transform->position[0] += speed * sim_dt;
    }
  }
}

void RuntimeHost::request_reload(const std::string& reason) {
  manual_reload_requested_ = true;
  manual_reload_reason_ = reason;
}

void RuntimeHost::force_reload(const std::string& reason) {
  reload_content(false, reason);
}

void RuntimeHost::handle_reload(bool manual_requested) {
  bool auto_reload_requested = false;
  if (project_.dev_mode) {
    const auto now = std::chrono::steady_clock::now();
    if (last_cook_check_ == std::chrono::steady_clock::time_point() ||
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cook_check_).count() > 200) {
      last_cook_check_ = now;
      const uint64_t status_mtime = file_mtime_epoch(cook_status_path_);
      if (status_mtime > 0 && status_mtime != cook_status_mtime_) {
        cook_status_mtime_ = status_mtime;
        CookStatus status;
        std::string error;
        if (load_cook_status(cook_status_path_, status, error)) {
          if (status.cook_ok) {
            if (!status.last_success.empty() && status.last_success != last_cook_success_) {
              last_cook_success_ = status.last_success;
            }
            auto_reload_requested = true;
          } else if (!status.last_error.empty() && status.last_error != last_cook_error_) {
            last_cook_error_ = status.last_error;
            rkg::log::warn(std::string("cook_status: ") + status.last_error);
          }
        } else if (!error.empty() && error != last_cook_error_) {
          last_cook_error_ = error;
          rkg::log::warn(std::string("cook_status read failed: ") + error);
        }
      }
    }
  }

  std::vector<rkg::platform::FileChange> changes;
  watcher_.poll(changes);
  const bool raw_changed = !changes.empty();
  bool reload_requested = manual_requested || raw_changed || auto_reload_requested;
#if RKG_ENABLE_IMGUI
  reload_requested = reload_requested || rkg::debug_ui::consume_force_reload_request();
#endif

  if (reload_requested) {
    const std::string reason = manual_requested && !manual_reload_reason_.empty() ? manual_reload_reason_ : "reload";
    reload_content(raw_changed, reason);
  }
}

bool RuntimeHost::reload_content(bool raw_changed, const std::string& reason) {
  const uint64_t cooked_before = cooked_mtime_;
  const uint64_t cooked_now = file_mtime_epoch(cooked_index_);
  if (cooked_now == 0) {
    cooked_available_ = false;
  } else if (cooked_now != cooked_mtime_) {
    cooked_mtime_ = cooked_now;
    cooked_available_ = true;
    using_cooked_ = true;
  }

  std::string reload_error;
  if (raw_changed && using_cooked_ && cooked_now <= cooked_before) {
    reload_error = "raw content changed; run rkgctl content cook";
  }

  bool can_reload = true;
  if (!cooked_available_) {
    if (project_.dev_mode) {
      using_cooked_ = false;
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
    const uint64_t pack_now = file_mtime_epoch(pack_path_);
    if (using_cooked_ && pack_now > 0 && pack_now != pack_mtime_) {
      pack_mtime_ = pack_now;
      std::string pack_error;
      if (pack_reader_.load(pack_path_, pack_error)) {
        pack_loaded_ = true;
        rkg::log::info("content pack reloaded");
      } else {
        pack_loaded_ = false;
        rkg::log::warn(std::string("content pack reload failed: ") + pack_error);
      }
    } else if (pack_now == 0) {
      pack_mtime_ = 0;
      pack_loaded_ = false;
    }

    reset_runtime();
    load_initial_level();
    rkg::log::info(std::string("content reloaded (") + (using_cooked_ ? "cooked" : "raw") + ")");
  }

  last_reload_time_ = now_iso();
  last_reload_error_ = reload_error;
#if RKG_ENABLE_IMGUI
  rkg::debug_ui::set_hot_reload_status(last_reload_time_, last_reload_error_);
#endif

  if (!reason.empty()) {
    rkg::log::info(std::string("reload reason: ") + reason);
  }

  return reload_error.empty();
}

void RuntimeHost::reset_runtime() {
  registry_ = rkg::ecs::Registry{};
  entities_by_name_.clear();
  entity_override_keys_.clear();
  entities_by_override_key_.clear();
  player_ = rkg::ecs::kInvalidEntity;
  current_level_path_.clear();
}

void RuntimeHost::load_initial_level() {
  if (project_.initial_level.empty()) {
    return;
  }
  current_level_path_ = project_.initial_level;
  rkg::content::PackReader* pack_ptr = (using_cooked_ && pack_loaded_) ? &pack_reader_ : nullptr;
  const fs::path content_root = using_cooked_ ? cooked_root_ : raw_content_root_;
  load_level(registry_, entities_by_name_, entity_override_keys_, entities_by_override_key_,
             player_, content_root, project_.initial_level, pack_ptr);
  apply_editor_overrides(registry_, entities_by_override_key_, project_root_ / "editor_overrides.yaml");
}

std::string RuntimeHost::renderer_display_name() const {
  return rkg::renderer_display_name(renderer_plugin_);
}

std::string RuntimeHost::override_key_for_entity(rkg::ecs::Entity entity) const {
  const auto it = entity_override_keys_.find(entity);
  if (it == entity_override_keys_.end()) {
    return {};
  }
  return it->second;
}

} // namespace rkg::runtime
