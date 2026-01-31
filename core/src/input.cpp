#include "rkg/input.h"

#include "rkg/log.h"

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

#if RKG_ENABLE_DATA_YAML
#include <yaml-cpp/yaml.h>
#endif

#include <algorithm>
#include <cctype>
#include <fstream>

namespace rkg::input {

namespace {
std::string to_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}
} // namespace

platform::KeyCode keycode_from_string(const std::string& name) {
  const auto key = to_lower(name);
  if (key == "w") return platform::KeyCode::W;
  if (key == "a") return platform::KeyCode::A;
  if (key == "s") return platform::KeyCode::S;
  if (key == "d") return platform::KeyCode::D;
  if (key == "space") return platform::KeyCode::Space;
  if (key == "leftshift" || key == "lshift" || key == "shift") return platform::KeyCode::LeftShift;
  if (key == "escape" || key == "esc") return platform::KeyCode::Escape;
  if (key == "f1") return platform::KeyCode::F1;
  if (key == "f5") return platform::KeyCode::F5;
  return platform::KeyCode::Unknown;
}

void InputMap::bind(const std::string& action, platform::KeyCode key) {
  bindings_[action] = key;
}

bool InputMap::load_from_file(const std::filesystem::path& path) {
  bindings_.clear();
  if (!std::filesystem::exists(path)) {
    rkg::log::warn(std::string("input map not found: ") + path.string());
    return false;
  }
  const auto ext = path.extension().string();
  if (ext == ".json") {
#if RKG_ENABLE_DATA_JSON
    nlohmann::json doc;
    {
      std::ifstream in(path);
      in >> doc;
    }
    const auto& root = doc.contains("input") ? doc["input"] : doc;
    if (root.contains("actions") && root["actions"].is_object()) {
      for (auto it = root["actions"].begin(); it != root["actions"].end(); ++it) {
        if (!it.value().is_string()) continue;
        bind(it.key(), keycode_from_string(it.value().get<std::string>()));
      }
    }
    return true;
#else
    rkg::log::warn("JSON input map requested but JSON support is disabled.");
    return false;
#endif
  }

  if (ext == ".yaml" || ext == ".yml") {
#if RKG_ENABLE_DATA_YAML
    YAML::Node doc = YAML::LoadFile(path.string());
    YAML::Node root = doc["input"] ? doc["input"] : doc;
    if (root["actions"]) {
      for (const auto& pair : root["actions"]) {
        const auto action = pair.first.as<std::string>();
        const auto key = pair.second.as<std::string>();
        bind(action, keycode_from_string(key));
      }
    }
    return true;
#else
    rkg::log::warn("YAML input map requested but YAML support is disabled.");
    return false;
#endif
  }

  rkg::log::warn("unknown input map extension");
  return false;
}

void InputSystem::set_map(const InputMap& map) {
  bindings_ = map.bindings();
  previous_.clear();
  states_.clear();
}

void InputSystem::update(const platform::Platform& platform) {
  // DEBUG: input key state tracing to diagnose missing movement. Remove once resolved.
  static int debug_frame = 0;
  const bool should_log = ((debug_frame++ % 60) == 0);

  for (const auto& kv : bindings_) {
    const auto& action = kv.first;
    const auto key = kv.second;
    const bool down = platform.is_key_down(key);
    const bool was_down = previous_[action];
    ActionState state;
    state.pressed = down && !was_down;
    state.released = !down && was_down;
    state.held = down;
    states_[action] = state;
    previous_[action] = down;

    if (should_log && down) {
      rkg::log::info("input: action=" + action + " key=" + std::to_string(static_cast<int>(key)) +
                     " down=" + std::to_string(down));
    }
  }
}

ActionState InputSystem::action(const std::string& name) const {
  auto it = states_.find(name);
  if (it == states_.end()) return {};
  return it->second;
}

} // namespace rkg::input
