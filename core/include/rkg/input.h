#pragma once

#include "rkg_platform/platform.h"

#include <filesystem>
#include <string>
#include <unordered_map>

namespace rkg::input {

struct ActionState {
  bool pressed = false;
  bool held = false;
  bool released = false;
};

class InputMap {
 public:
  bool load_from_file(const std::filesystem::path& path);
  void bind(const std::string& action, platform::KeyCode key);
  const std::unordered_map<std::string, platform::KeyCode>& bindings() const { return bindings_; }

 private:
  std::unordered_map<std::string, platform::KeyCode> bindings_;
};

class InputSystem {
 public:
  void set_map(const InputMap& map);
  void update(const platform::Platform& platform);
  ActionState action(const std::string& name) const;
  const std::unordered_map<std::string, ActionState>& states() const { return states_; }

 private:
  std::unordered_map<std::string, platform::KeyCode> bindings_;
  std::unordered_map<std::string, bool> previous_;
  std::unordered_map<std::string, ActionState> states_;
};

platform::KeyCode keycode_from_string(const std::string& name);

} // namespace rkg::input
