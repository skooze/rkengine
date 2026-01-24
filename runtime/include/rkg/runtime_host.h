#pragma once

#include "rkg/ecs.h"
#include "rkg/content_pack.h"
#include "rkg/input.h"
#include "rkg/host_context.h"
#include "rkg/paths.h"
#include "rkg/plugin_host.h"
#include "rkg/project.h"
#include "rkg_platform/platform.h"
#include "rkg_platform/file_watcher.h"

#include <chrono>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace rkg::runtime {

struct RuntimeHostInit {
  const char* argv0 = nullptr;
  std::optional<std::filesystem::path> project_override;
  std::string default_project = "demo_game";
  std::string app_name = "rkg_app";
  rkg::platform::WindowDesc window{};
  bool force_debug_ui = false;
  bool disable_debug_ui = false;
};

struct FrameParams {
  float frame_dt = 0.0f;
  float sim_dt = 0.0f;
  bool run_simulation = true;
  bool update_input = true;
};

class RuntimeHost {
 public:
  using ActionStateProvider = std::function<rkg::input::ActionState(const std::string&)>;

  bool init(const RuntimeHostInit& init, std::string& error);
  void shutdown();

  void tick(const FrameParams& params, const ActionStateProvider& action_state_provider = ActionStateProvider{});
  void update_input();
  rkg::input::ActionState input_action(const std::string& name) const;

  void request_reload(const std::string& reason = "");
  void force_reload(const std::string& reason = "");

  rkg::platform::Platform& platform() { return platform_; }
  const rkg::platform::Platform& platform() const { return platform_; }

  const rkg::ResolvedPaths& paths() const { return paths_; }
  const std::filesystem::path& executable_dir() const { return executable_dir_; }
  const rkg::ProjectConfig& project() const { return project_; }
  const std::string& project_name() const { return project_name_; }
  const std::filesystem::path& project_root() const { return project_root_; }
  const std::filesystem::path& raw_content_root() const { return raw_content_root_; }
  const std::filesystem::path& cooked_root() const { return cooked_root_; }
  const std::filesystem::path& pack_path() const { return pack_path_; }
  const std::filesystem::path& cook_status_path() const { return cook_status_path_; }
  const std::string& current_level_path() const { return current_level_path_; }

  const std::string& renderer_plugin() const { return renderer_plugin_; }
  uint32_t renderer_caps() const { return renderer_caps_; }
  std::string renderer_display_name() const;

  bool debug_ui_enabled() const { return debug_ui_enabled_; }
  bool debug_ui_requested() const { return debug_ui_requested_; }
  const std::string& debug_ui_unavailable_reason() const { return debug_ui_unavailable_reason_; }

  bool using_cooked() const { return using_cooked_; }
  bool cooked_available() const { return cooked_available_; }
  const std::string& last_cook_success() const { return last_cook_success_; }
  const std::string& last_cook_error() const { return last_cook_error_; }
  const std::string& last_reload_time() const { return last_reload_time_; }
  const std::string& last_reload_error() const { return last_reload_error_; }

  const rkg::ecs::Registry& registry() const { return registry_; }
  rkg::ecs::Entity player_entity() const { return player_; }
  const std::unordered_map<std::string, rkg::ecs::Entity>& entities_by_name() const { return entities_by_name_; }
  const std::unordered_map<rkg::ecs::Entity, std::string>& entity_override_keys() const {
    return entity_override_keys_;
  }
  std::string override_key_for_entity(rkg::ecs::Entity entity) const;
  const rkg::input::InputSystem& input() const { return input_; }
  rkg::input::InputSystem& input() { return input_; }

 private:
  void setup_plugins(bool force_debug_ui, bool disable_debug_ui, std::string& error);
  void handle_reload(bool manual_requested);
  bool reload_content(bool raw_changed, const std::string& reason);
  void reset_runtime();
 void load_initial_level();
 static void sdl_event_callback(const void* event, void* user_data);

  rkg::ResolvedPaths paths_{};
  std::filesystem::path executable_dir_;
  std::filesystem::path project_root_;
  std::string project_name_;
  rkg::ProjectConfig project_{};

  rkg::platform::Platform platform_{};

  rkg::input::InputMap input_map_{};
  rkg::input::InputSystem input_{};

  rkg::ecs::Registry registry_{};
  std::unordered_map<std::string, rkg::ecs::Entity> entities_by_name_{};
  std::unordered_map<rkg::ecs::Entity, std::string> entity_override_keys_{};
  std::unordered_map<std::string, rkg::ecs::Entity> entities_by_override_key_{};
  rkg::ecs::Entity player_ = rkg::ecs::kInvalidEntity;

  std::filesystem::path raw_content_root_;
  std::filesystem::path cooked_root_;
  std::filesystem::path cooked_index_;
  std::filesystem::path cook_status_path_;
  std::filesystem::path pack_path_;
  bool using_cooked_ = false;
  bool cooked_available_ = false;
  bool pack_loaded_ = false;
  rkg::content::PackReader pack_reader_{};
  uint64_t cooked_mtime_ = 0;
  uint64_t pack_mtime_ = 0;
  uint64_t cook_status_mtime_ = 0;
  std::string last_cook_success_;
  std::string last_cook_error_;
  std::chrono::steady_clock::time_point last_cook_check_{};
  std::string last_reload_time_ = "N/A";
  std::string last_reload_error_;
  std::string current_level_path_;

  bool manual_reload_requested_ = false;
  std::string manual_reload_reason_;

  std::string renderer_plugin_;
  uint32_t renderer_caps_ = 0;
  std::vector<std::string> active_plugins_;
  bool debug_ui_enabled_ = false;
  bool debug_ui_requested_ = false;
  std::string debug_ui_unavailable_reason_;

  rkg::HostContext host_ctx_{};
  rkg::PluginHost host_{};
  rkg::platform::FileWatcher watcher_{};
};

} // namespace rkg::runtime
