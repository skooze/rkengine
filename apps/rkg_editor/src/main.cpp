#include "rkg/runtime_host.h"
#include "rkg/run_cleanup.h"
#include "rkg/snapshot_restore.h"
#include "rkg/staged_runs.h"

#include "subprocess_runner.h"

#include "rkg/log.h"
#include "rkg/math.h"
#include "rkg/paths.h"
#include "rkg/renderer_hooks.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <limits>
#include <filesystem>
#include <fstream>
#include <cstring>
#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#if RKG_ENABLE_IMGUI && RKG_ENABLE_VULKAN
#include "rkg_debug_ui/imgui_api.h"
#include <imgui.h>
#include <imgui_internal.h>
#define RKG_EDITOR_IMGUI 1
#else
#define RKG_EDITOR_IMGUI 0
#endif

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

#if RKG_ENABLE_DATA_YAML
#include <yaml-cpp/yaml.h>
#endif

namespace fs = std::filesystem;

namespace {
using rkg::Mat4;
using rkg::Vec3;
using rkg::mat4_identity;
using rkg::mat4_look_at;
using rkg::mat4_mul;
using rkg::mat4_perspective;
using rkg::mat4_rotation_x;
using rkg::mat4_rotation_y;
using rkg::mat4_rotation_xyz;
using rkg::mat4_rotation_z;
using rkg::mat4_scale;
using rkg::mat4_translation;
using rkg::vec3_add;
using rkg::vec3_cross;
using rkg::vec3_dot;
using rkg::vec3_mul;
using rkg::vec3_normalize;
using rkg::vec3_sub;

enum class PlayState {
  Edit,
  Play,
  Pause
};

struct PlanSummary {
  bool valid = false;
  size_t task_count = 0;
  std::vector<std::string> top_tasks;
  std::string plan_path;
  std::string error;
};

struct AgentRunSummary {
  bool valid = false;
  std::string run_id;
  std::string run_dir;
  std::string created_at;
  std::string goal;
  std::string mode;
  std::string provider;
  std::string model;
  std::string plan_path;
  std::string status;
  bool dry_run = false;
  int conflicts = 0;
  bool drift_detected = false;
  int drift_added = 0;
  int drift_removed = 0;
  int drift_modified = 0;
  std::string drift_message;
  std::string error_code;
  std::string error_message;
  std::string error_stage;
  std::string selector_type;
  std::string selector_value;
  std::string selector_warning;
  std::string entity_id;
  std::string entity_name;
  std::vector<std::string> conflict_files;
};

struct AgentStatusSnapshot {
  bool valid = false;
  std::string provider;
  std::string provider_note;
  std::string model;
  std::string base_url;
  std::string templates_dir;
  int timeout_seconds = 0;
  AgentRunSummary last_run;
  std::string parse_error;
};

struct AgentPanelState {
  std::string goal;
  std::string offline_plan_path;
  std::string plan_path;
  char goal_buffer[1024]{};
  char plan_path_buffer[512]{};
  char offline_plan_buffer[512]{};
  PlanSummary plan_summary;
  AgentStatusSnapshot status;
  std::string last_error;
  bool openai_available = false;
  std::string openai_error;

  enum class Stage { Idle, RunningAgent, RunningStatus } stage = Stage::Idle;
  rkg::editor::SubprocessRunner agent_process;
  rkg::editor::SubprocessRunner status_process;
  rkg::editor::SubprocessRunner open_process;
  std::string open_error;
  std::string last_command_label;
};

struct ContentPanelState {
  rkg::editor::SubprocessRunner cook_process;
  rkg::editor::SubprocessRunner watch_process;
  rkg::editor::SubprocessRunner commit_process;
  rkg::editor::SubprocessRunner open_file_process;
  std::string open_file_error;
  int commit_last_exit = -1;
  std::string commit_error;
  std::string last_error;
  std::string commit_run_id;
  std::string commit_run_dir;
  std::string commit_staging_dir;
  std::string commit_results_path;
  std::string commit_status;
  std::string commit_stage;
  std::string commit_error_code;
  std::string commit_error_message;
  std::string commit_entity_id;
  std::string commit_selector_type;
  std::string commit_selector_value;
  std::string commit_selector_warning;
  bool commit_dry_run = true;
  int commit_conflicts = 0;
  std::vector<std::string> commit_conflict_files;
  bool commit_conflict_detected = false;
  bool commit_forced_apply = false;
  bool commit_snapshots_taken = false;
  std::string commit_snapshot_manifest;
  std::string commit_handled_run_id;
  bool commit_clear_pending = false;
  bool commit_stage_selected_only = false;
  bool commit_force_modal = false;
};

struct DiffPreviewEntry {
  std::string target_path;
  std::string diff_path;
  std::string diff_text;
  bool truncated = false;
};

struct DiffPreviewState {
  std::string run_id;
  std::string run_dir;
  std::string level_path;
  std::string entity_id;
  std::string selector_type;
  std::string selector_value;
  std::string selector_warning;
  std::vector<DiffPreviewEntry> diffs;
  int selected = 0;
  bool loaded = false;
  std::string error;
};

struct SnapshotEntry {
  std::string path;
  std::string hash;
  std::string timestamp;
  std::string snapshot_path;
  uint64_t size = 0;
  bool missing = false;
};

struct RunManifest {
  std::string run_id;
  std::string run_type;
  std::string timestamp;
  std::string project_path;
  std::string goal;
  std::string status;
  bool success = false;
  bool incomplete = false;
  std::string error_code;
  std::string error_message;
  std::string error_stage;
  std::string run_dir;
  std::string staged_patches;
  std::vector<std::string> target_files;
  std::vector<std::string> conflict_files;
  bool snapshots_taken = false;
  std::string snapshot_manifest_path;
  std::vector<SnapshotEntry> snapshots;
  std::string selector_type;
  std::string selector_value;
  std::string selector_warning;
  std::string overrides_path;
  std::string level_path;
  std::string entity_id;
  std::string entity_name;
  std::string snapshot_run_id;
  std::string snapshot_path;
  std::chrono::system_clock::time_point sort_time{};
};

struct RunsBrowserState {
  std::vector<RunManifest> runs;
  int selected = 0;
  bool loaded = false;
  std::chrono::steady_clock::time_point last_refresh{};
  rkg::editor::SubprocessRunner restage_process;
  rkg::editor::SubprocessRunner open_process;
  rkg::editor::SubprocessRunner open_file_process;
  bool restage_pending = false;
  std::string restage_error;
  std::string restage_run_dir;
  std::string restage_output_run_dir;
  std::string restage_output_error;
  std::string open_error;
  std::string open_file_error;
  std::string selected_run_dir;
  std::vector<DiffPreviewEntry> diffs;
  int diff_selected = 0;
  bool diffs_loaded = false;
  std::string diff_error;
  std::string runs_error;
  std::string diff_title;
  bool diff_is_snapshot = false;

  char search_buffer[256]{};
  bool filter_ai = true;
  bool filter_commit = true;
  bool filter_snapshot = true;
  bool filter_other = true;
  bool status_success = true;
  bool status_failed = true;
  bool status_conflict = true;
  bool status_unknown = true;

  bool cleanup_modal = false;
  int cleanup_keep_last = 50;
  int cleanup_days = 14;
  std::vector<std::filesystem::path> cleanup_candidates;
  std::string cleanup_error;

  bool restore_apply_modal = false;
  std::string restore_apply_run_dir;
  std::string restore_error;
};

struct EntitySnapshot {
  bool has_transform = false;
  rkg::ecs::Transform transform{};
  bool has_renderable = false;
  rkg::ecs::Renderable renderable{};
};

struct OverridesState {
  fs::path path;
  std::unordered_map<std::string, EntitySnapshot> saved;
  std::string last_reload_time;
  std::string last_error;
  bool dirty = false;
};

struct UndoEntry {
  rkg::ecs::Entity entity = rkg::ecs::kInvalidEntity;
  EntitySnapshot before;
  EntitySnapshot after;
};

struct UndoState {
  bool active = false;
  uint32_t active_id = 0;
  rkg::ecs::Entity active_entity = rkg::ecs::kInvalidEntity;
  EntitySnapshot before;
  std::vector<UndoEntry> undo;
  std::vector<UndoEntry> redo;
};

struct EditorState {
  rkg::runtime::RuntimeHost* runtime = nullptr;
  PlayState play_state = PlayState::Edit;
  bool step_requested = false;
  bool stop_requested = false;
  float fixed_step = 1.0f / 60.0f;
  bool dock_built = false;
  bool viewport_focused = false;
  bool ui_capturing = false;
  bool chat_active = false;
  bool auto_select = true;
  bool pick_requested = false;
  bool diff_autoload_done = false;
  float viewport_pos[2]{0.0f, 0.0f};
  float viewport_size[2]{0.0f, 0.0f};
  float pick_mouse_pos[2]{0.0f, 0.0f};
  float camera_eye[3]{0.0f, 0.0f, 0.0f};
  float camera_forward[3]{0.0f, 0.0f, 1.0f};
  float camera_right[3]{1.0f, 0.0f, 0.0f};
  float camera_up[3]{0.0f, 1.0f, 0.0f};
  float camera_yaw = 0.0f;
  float camera_pitch = -0.3f;
  float camera_distance = 4.0f;
  float camera_fov = 60.0f;
  float camera_pan[3]{0.0f, 0.0f, 0.0f};
  float editor_pivot_world[3]{0.0f, 0.0f, 0.0f};
  bool lock_editor_pivot = false;
  float camera_view_proj[16]{};
  struct SavedEditorCamera {
    bool valid = false;
    float yaw = 0.0f;
    float pitch = 0.0f;
    float distance = 0.0f;
    float pan[3]{0.0f, 0.0f, 0.0f};
    float pivot[3]{0.0f, 0.0f, 0.0f};
  } saved_editor_cam;
  bool show_world_grid = true;
  bool show_character_grid = true;
  bool show_world_axes = true;
  bool show_face_labels = true;
  bool show_skeleton_debug = true;
  float grid_half_extent = 10.0f;
  float grid_step = 1.0f;

  rkg::ecs::Entity selected_entity = rkg::ecs::kInvalidEntity;
  std::string selected_name;
  rkg::ecs::Entity fallback_entity = rkg::ecs::kInvalidEntity;
  int selected_bone = -1;

  AgentPanelState agent;
  ContentPanelState content;
  DiffPreviewState diff_preview;
  RunsBrowserState runs;
  OverridesState overrides;
  UndoState undo;
};

std::string join_lines(const std::vector<std::string>& lines) {
  std::ostringstream out;
  for (size_t i = 0; i < lines.size(); ++i) {
    out << lines[i];
    if (i + 1 < lines.size()) out << "\n";
  }
  return out.str();
}

std::string read_text_file(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string read_text_file_limited(const fs::path& path, size_t max_chars, bool& truncated) {
  truncated = false;
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  std::string out;
  out.reserve(max_chars);
  char buffer[4096];
  size_t total = 0;
  while (in && total < max_chars) {
    const size_t remaining = max_chars - total;
    const size_t chunk = std::min<size_t>(sizeof(buffer), remaining);
    in.read(buffer, static_cast<std::streamsize>(chunk));
    const std::streamsize got = in.gcount();
    if (got <= 0) break;
    out.append(buffer, static_cast<size_t>(got));
    total += static_cast<size_t>(got);
  }
  if (in && in.peek() != std::char_traits<char>::eof()) {
    truncated = true;
  }
  return out;
}

std::vector<std::string> split_lines(const std::string& text) {
  std::vector<std::string> lines;
  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    lines.push_back(line);
  }
  if (!text.empty() && text.back() == '\n') {
    lines.push_back("");
  }
  return lines;
}

std::string unified_diff_text(const std::string& old_text,
                              const std::string& new_text,
                              const std::string& path) {
  const auto old_lines = split_lines(old_text);
  const auto new_lines = split_lines(new_text);
  const size_t n = old_lines.size();
  const size_t m = new_lines.size();

  std::vector<std::vector<int>> lcs(n + 1, std::vector<int>(m + 1, 0));
  for (size_t i = n; i-- > 0;) {
    for (size_t j = m; j-- > 0;) {
      if (old_lines[i] == new_lines[j]) {
        lcs[i][j] = lcs[i + 1][j + 1] + 1;
      } else {
        lcs[i][j] = std::max(lcs[i + 1][j], lcs[i][j + 1]);
      }
    }
  }

  std::ostringstream diff;
  diff << "--- a/" << path << "\n";
  diff << "+++ b/" << path << "\n";
  diff << "@@ -" << 1 << "," << n << " +" << 1 << "," << m << " @@\n";

  size_t i = 0;
  size_t j = 0;
  while (i < n || j < m) {
    if (i < n && j < m && old_lines[i] == new_lines[j]) {
      diff << " " << old_lines[i] << "\n";
      ++i;
      ++j;
    } else if (j < m && (i == n || lcs[i][j + 1] >= lcs[i + 1][j])) {
      diff << "+" << new_lines[j] << "\n";
      ++j;
    } else if (i < n) {
      diff << "-" << old_lines[i] << "\n";
      ++i;
    }
  }
  return diff.str();
}

struct Ray {
  Vec3 origin;
  Vec3 dir;
};

bool ray_intersect_aabb(const Ray& ray, const Vec3& min, const Vec3& max, float& out_t) {
  float tmin = 0.0f;
  float tmax = std::numeric_limits<float>::max();
  const float eps = 1e-6f;

  const float origin[3] = {ray.origin.x, ray.origin.y, ray.origin.z};
  const float dir[3] = {ray.dir.x, ray.dir.y, ray.dir.z};
  const float bmin[3] = {min.x, min.y, min.z};
  const float bmax[3] = {max.x, max.y, max.z};

  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(dir[axis]) < eps) {
      if (origin[axis] < bmin[axis] || origin[axis] > bmax[axis]) {
        return false;
      }
      continue;
    }
    const float inv = 1.0f / dir[axis];
    float t1 = (bmin[axis] - origin[axis]) * inv;
    float t2 = (bmax[axis] - origin[axis]) * inv;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    if (tmax < tmin) {
      return false;
    }
  }
  out_t = tmin;
  return true;
}

bool openai_available(std::string& error) {
#if !RKG_ENABLE_OPENAI
  error = "OpenAI provider is disabled in this build.";
  return false;
#else
  const char* key = std::getenv("OPENAI_API_KEY");
  if (!key || !*key) {
    error = "OPENAI_API_KEY is not set. Export the key or use Offline Apply.";
    return false;
  }
  return true;
#endif
}

#if RKG_ENABLE_DATA_JSON
bool load_json_file(const fs::path& path, nlohmann::json& out, std::string* error = nullptr) {
  if (!fs::exists(path)) {
    if (error) *error = "file not found: " + path.generic_string();
    return false;
  }
  std::ifstream in(path);
  if (!in) {
    if (error) *error = "file open failed: " + path.generic_string();
    return false;
  }
  in >> out;
  return true;
}

PlanSummary load_plan_summary(const fs::path& plan_path) {
  PlanSummary summary;
  summary.plan_path = plan_path.generic_string();
  if (!fs::exists(plan_path)) {
    summary.error = "plan not found";
    return summary;
  }
  std::ifstream in(plan_path);
  if (!in) {
    summary.error = "plan open failed";
    return summary;
  }
  nlohmann::json plan;
  in >> plan;
  if (!plan.contains("tasks") || !plan["tasks"].is_array()) {
    summary.error = "tasks missing";
    return summary;
  }
  const auto& tasks = plan["tasks"];
  summary.task_count = tasks.size();
  const size_t max_tasks = 5;
  for (size_t i = 0; i < tasks.size() && i < max_tasks; ++i) {
    const auto& task = tasks[i];
    std::string title = task.value("title", "");
    if (title.empty()) {
      title = task.value("id", "");
    }
    if (title.empty()) {
      title = task.value("type", "task");
    }
    summary.top_tasks.push_back(title);
  }
  summary.valid = true;
  return summary;
}

AgentStatusSnapshot parse_agent_status_json(const std::string& text, const fs::path& root) {
  AgentStatusSnapshot out;
  nlohmann::json payload = nlohmann::json::parse(text, nullptr, false);
  if (payload.is_discarded()) {
    out.parse_error = "agent status parse failed";
    return out;
  }

  const auto config = payload.value("config", nlohmann::json::object());
  out.provider = config.value("provider", "");
  out.provider_note = config.value("provider_note", "");
  out.model = config.value("model", "");
  out.base_url = config.value("base_url", "");
  out.templates_dir = config.value("templates_dir", "");
  out.timeout_seconds = config.value("timeout_seconds", 0);

  const auto runs = payload.value("runs", nlohmann::json::array());
  if (!runs.empty()) {
    const auto run = runs[0];
    out.last_run.run_id = run.value("run_id", "");
    out.last_run.created_at = run.value("created_at", "");
    out.last_run.goal = run.value("goal", "");
    out.last_run.mode = run.value("mode", "");
    out.last_run.provider = run.value("provider", "");
    out.last_run.model = run.value("model", "");
    out.last_run.plan_path = run.value("plan_path", "");
    if (!out.last_run.run_id.empty()) {
      out.last_run.run_dir = (root / "build" / "ai_runs" / out.last_run.run_id).generic_string();
    }

    const auto results = run.value("results", nlohmann::json::object());
    out.last_run.status = results.value("status", "");
    out.last_run.dry_run = results.value("dry_run", false);
    out.last_run.conflicts = results.value("conflicts", 0);
    out.last_run.error_code = results.value("error_code", "");
    out.last_run.error_message = results.value("error_message", "");
    out.last_run.error_stage = results.value("stage", "");

    nlohmann::json drift = nlohmann::json::object();
    if (run.contains("context_drift")) {
      drift = run["context_drift"];
    } else if (results.contains("context_drift")) {
      drift = results["context_drift"];
    }
    out.last_run.drift_detected = drift.value("drift_detected", false);
    out.last_run.drift_added = drift.value("drift_added", 0);
    out.last_run.drift_removed = drift.value("drift_removed", 0);
    out.last_run.drift_modified = drift.value("drift_modified", 0);
    out.last_run.drift_message = drift.value("drift_message", "");
    out.last_run.valid = true;
  }

  out.valid = true;
  return out;
}

bool load_commit_overrides_summary(const fs::path& root, ContentPanelState& content, std::string& error) {
  error.clear();
  const fs::path marker_path = root / "build" / "ai_runs" / "last_commit_overrides.json";
  if (!fs::exists(marker_path)) {
    error = "commit summary not found";
    return false;
  }
  std::ifstream in(marker_path);
  if (!in) {
    error = "commit summary open failed";
    return false;
  }
  nlohmann::json marker;
  in >> marker;
  content.commit_run_id = marker.value("run_id", "");
  content.commit_run_dir = marker.value("run_dir", "");
  content.commit_staging_dir = marker.value("staging_dir", "");
  const std::string results_path = marker.value("results_path", "");
  if (results_path.empty()) {
    error = "commit results path missing";
    return false;
  }
  content.commit_results_path = results_path;
  if (!fs::exists(results_path)) {
    error = "commit results not found";
    return false;
  }
  std::ifstream rin(results_path);
  if (!rin) {
    error = "commit results open failed";
    return false;
  }
  nlohmann::json results;
  rin >> results;
  content.commit_status = results.value("status", "");
  content.commit_stage = results.value("stage", "");
  content.commit_error_code = results.value("error_code", "");
  content.commit_error_message = results.value("error_message", "");
  content.commit_entity_id = results.value("entity_id", "");
  content.commit_selector_type = results.value("selector_type", "");
  content.commit_selector_value = results.value("selector_value", "");
  content.commit_selector_warning = results.value("selector_warning", "");
  content.commit_dry_run = results.value("dry_run", true);
  content.commit_conflicts = results.value("conflicts", 0);
  content.commit_conflict_files.clear();
  if (results.contains("conflict_files") && results["conflict_files"].is_array()) {
    for (const auto& item : results["conflict_files"]) {
      if (item.is_string()) {
        content.commit_conflict_files.push_back(item.get<std::string>());
      }
    }
  }
  content.commit_conflict_detected = results.value("conflict_detected", false);
  content.commit_forced_apply = results.value("forced_apply", false);
  content.commit_snapshots_taken = results.value("snapshots_taken", false);
  content.commit_snapshot_manifest = results.value("snapshot_manifest_path", "");
  return true;
}

std::string parse_diff_target_from_text(const std::string& text) {
  size_t start = text.find("\n+++ ");
  if (start == std::string::npos) {
    if (text.rfind("+++ ", 0) == 0) {
      start = 0;
    } else {
      return {};
    }
  } else {
    start += 1;
  }
  size_t line_end = text.find('\n', start);
  if (line_end == std::string::npos) {
    line_end = text.size();
  }
  const std::string line = text.substr(start, line_end - start);
  if (line.rfind("+++ ", 0) != 0) {
    return {};
  }
  std::string path = line.substr(4);
  if (path.rfind("b/", 0) == 0 || path.rfind("a/", 0) == 0) {
    path = path.substr(2);
  }
  return path;
}

bool load_diff_entries_from_dir(const fs::path& staged_dir,
                                const std::unordered_map<std::string, std::string>& diff_targets,
                                std::vector<DiffPreviewEntry>& out,
                                std::string& error,
                                size_t max_chars = 20000) {
  error.clear();
  out.clear();
  if (staged_dir.empty() || !fs::exists(staged_dir)) {
    error = "staged patches missing";
    return false;
  }

  std::vector<fs::path> diff_files;
  std::error_code ec;
  for (auto it = fs::recursive_directory_iterator(staged_dir, ec); it != fs::recursive_directory_iterator(); ++it) {
    if (ec) break;
    if (!it->is_regular_file()) continue;
    if (it->path().extension() == ".diff") {
      diff_files.push_back(it->path());
    }
  }
  std::sort(diff_files.begin(), diff_files.end());

  for (const auto& path : diff_files) {
    DiffPreviewEntry entry;
    entry.diff_path = path.generic_string();
    bool truncated = false;
    entry.diff_text = read_text_file_limited(path, max_chars, truncated);
    entry.truncated = truncated;
    auto it = diff_targets.find(entry.diff_path);
    if (it != diff_targets.end()) {
      entry.target_path = it->second;
    } else {
      entry.target_path = parse_diff_target_from_text(entry.diff_text);
      if (entry.target_path.empty()) {
        entry.target_path = path.filename().generic_string();
      }
    }
    out.push_back(entry);
  }
  return true;
}

bool load_commit_diff_preview(const fs::path& run_dir, DiffPreviewState& preview, std::string& error) {
  error.clear();
  preview.error.clear();
  preview.diffs.clear();
  preview.level_path.clear();
  preview.loaded = false;
  preview.run_dir = run_dir.generic_string();
  preview.run_id = run_dir.filename().string();
  preview.selected = 0;

  if (run_dir.empty()) {
    error = "staged run missing";
    return false;
  }

  const fs::path summary_path = run_dir / "staged_patches" / "summary.json";
  if (!fs::exists(summary_path)) {
    error = "staged summary missing";
    return false;
  }
  std::ifstream in(summary_path);
  if (!in) {
    error = "staged summary open failed";
    return false;
  }
  nlohmann::json summary;
  in >> summary;
  preview.run_id = summary.value("run_id", preview.run_id);
  preview.level_path = summary.value("level_path", "");
  preview.entity_id = summary.value("entity_id", "");
  preview.selector_type = summary.value("selector_type", "");
  preview.selector_value = summary.value("selector_value", "");
  preview.selector_warning = summary.value("selector_warning", "");

  std::unordered_map<std::string, std::string> diff_targets;
  const auto files = summary.value("files", nlohmann::json::array());
  for (const auto& file : files) {
    const std::string diff_path = file.value("diff_path", "");
    const std::string target = file.value("path", "");
    if (!diff_path.empty() && !target.empty()) {
      diff_targets[diff_path] = target;
    }
  }

  std::string diff_err;
  if (!load_diff_entries_from_dir(run_dir / "staged_patches", diff_targets, preview.diffs, diff_err)) {
    error = diff_err;
    return false;
  }
  preview.loaded = true;
  return true;
}

std::chrono::system_clock::time_point to_system_clock(fs::file_time_type ftime) {
  using namespace std::chrono;
  return time_point_cast<system_clock::duration>(
      ftime - fs::file_time_type::clock::now() + system_clock::now());
}

std::string format_timestamp(const std::chrono::system_clock::time_point& tp) {
  std::time_t time = std::chrono::system_clock::to_time_t(tp);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &time);
#else
  localtime_r(&time, &tm);
#endif
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
  return out.str();
}

bool load_snapshot_manifest(const fs::path& path, std::vector<SnapshotEntry>& out, std::string& error) {
  out.clear();
  error.clear();
  if (!fs::exists(path)) {
    error = "snapshot manifest missing";
    return false;
  }
  nlohmann::json manifest;
  if (!load_json_file(path, manifest, &error)) {
    return false;
  }
  if (!manifest.contains("files") || !manifest["files"].is_array()) {
    error = "snapshot manifest invalid";
    return false;
  }
  for (const auto& item : manifest["files"]) {
    SnapshotEntry entry;
    entry.path = item.value("path", "");
    entry.hash = item.value("hash", "");
    entry.timestamp = item.value("timestamp", "");
    entry.snapshot_path = item.value("snapshot_path", "");
    entry.size = static_cast<uint64_t>(item.value("size", 0));
    entry.missing = item.value("missing", false);
    out.push_back(entry);
  }
  return true;
}

bool build_snapshot_compare_diff(const SnapshotEntry& entry,
                                 const fs::path& root,
                                 DiffPreviewEntry& out,
                                 bool& current_missing,
                                 std::string& error) {
  error.clear();
  current_missing = false;
  fs::path snapshot_path = entry.snapshot_path;
  if (snapshot_path.empty()) {
    error = "snapshot path missing";
    return false;
  }
  if (!snapshot_path.is_absolute()) {
    snapshot_path = root / snapshot_path;
  }
  if (!fs::exists(snapshot_path)) {
    error = "snapshot file missing";
    return false;
  }
  const std::string snapshot_text = read_text_file(snapshot_path);

  fs::path current_path = entry.path;
  if (!current_path.is_absolute()) {
    current_path = root / current_path;
  }
  std::string current_text;
  if (fs::exists(current_path)) {
    current_text = read_text_file(current_path);
  } else {
    current_missing = true;
    current_text.clear();
  }

  out.target_path = entry.path;
  out.diff_path = "(snapshot)";
  out.diff_text = unified_diff_text(snapshot_text, current_text,
                                    entry.path.empty() ? current_path.generic_string() : entry.path);
  out.truncated = false;
  return true;
}

void collect_summary_targets(const nlohmann::json& summary,
                             std::vector<std::string>& target_files,
                             std::unordered_map<std::string, std::string>& diff_targets) {
  target_files.clear();
  diff_targets.clear();
  const auto files = summary.value("files", nlohmann::json::array());
  for (const auto& file : files) {
    const std::string diff_path = file.value("diff_path", "");
    const std::string target = file.value("path", "");
    if (!target.empty()) {
      target_files.push_back(target);
    }
    if (!diff_path.empty() && !target.empty()) {
      diff_targets[diff_path] = target;
    }
  }
  std::sort(target_files.begin(), target_files.end());
  target_files.erase(std::unique(target_files.begin(), target_files.end()), target_files.end());
}

bool load_run_manifest(const fs::path& run_dir, RunManifest& out) {
  out = RunManifest{};
  out.run_dir = run_dir.generic_string();
  out.run_id = run_dir.filename().string();

  nlohmann::json run_info;
  nlohmann::json results;
  nlohmann::json summary;
  const bool has_info = load_json_file(run_dir / "run_info.json", run_info, nullptr);
  const bool has_results = load_json_file(run_dir / "results.json", results, nullptr);
  const bool has_summary = load_json_file(run_dir / "staged_patches" / "summary.json", summary, nullptr);

  if (!has_info || !has_results) {
    out.incomplete = true;
  }

  std::string mode;
  std::string provider;
  if (has_info) {
    out.run_id = run_info.value("run_id", out.run_id);
    out.timestamp = run_info.value("created_at", "");
    out.goal = run_info.value("goal", "");
    mode = run_info.value("mode", "");
    provider = run_info.value("provider", "");
    out.overrides_path = run_info.value("overrides_path", "");
    out.level_path = run_info.value("level_path", "");
    out.entity_id = run_info.value("entity_id", "");
    out.entity_name = run_info.value("entity_name", "");
    out.snapshot_run_id = run_info.value("snapshot_run_id", "");
    out.snapshot_path = run_info.value("snapshot_path", "");
  }

  if (has_results) {
    out.status = results.value("status", "");
    out.error_code = results.value("error_code", "");
    out.error_message = results.value("error_message", "");
    out.error_stage = results.value("stage", "");
    out.success = results.value("success", out.status == "ok");
    out.conflict_files.clear();
    if (results.contains("conflict_files") && results["conflict_files"].is_array()) {
      for (const auto& item : results["conflict_files"]) {
        if (item.is_string()) out.conflict_files.push_back(item.get<std::string>());
      }
    }
    out.snapshots_taken = results.value("snapshots_taken", false);
    out.snapshot_manifest_path = results.value("snapshot_manifest_path", "");
    out.selector_type = results.value("selector_type", "");
    out.selector_value = results.value("selector_value", "");
    out.selector_warning = results.value("selector_warning", "");
  }

  if (has_summary) {
    if (out.level_path.empty()) {
      out.level_path = summary.value("level_path", "");
    }
    if (out.selector_type.empty()) {
      out.selector_type = summary.value("selector_type", "");
    }
    if (out.selector_value.empty()) {
      out.selector_value = summary.value("selector_value", "");
    }
    if (out.selector_warning.empty()) {
      out.selector_warning = summary.value("selector_warning", "");
    }
    if (out.entity_id.empty()) {
      out.entity_id = summary.value("entity_id", "");
    }
    if (out.entity_name.empty()) {
      out.entity_name = summary.value("entity_name", "");
    }
    std::unordered_map<std::string, std::string> diff_targets;
    collect_summary_targets(summary, out.target_files, diff_targets);
  }

  const fs::path staged_dir = run_dir / "staged_patches";
  if (fs::exists(staged_dir)) {
    out.staged_patches = staged_dir.generic_string();
  }

  if (mode == "commit_overrides") {
    const std::string stage = has_results ? results.value("stage", "") : "";
    out.run_type = (stage == "apply") ? "commit_overrides_apply" : "commit_overrides_stage";
  } else if (mode == "snapshot_restore") {
    const std::string stage = has_results ? results.value("stage", "") : "";
    out.run_type = (stage == "apply") ? "snapshot_restore_apply" : "snapshot_restore_stage";
  } else if (mode == "plan") {
    out.run_type = "agent_plan";
  } else if (mode == "apply") {
    out.run_type = (provider == "offline") ? "offline_apply" : "agent_apply";
  } else {
    out.run_type = "unknown";
  }

  if (out.timestamp.empty()) {
    std::error_code ec;
    const auto time_path = fs::exists(run_dir / "run_info.json") ? (run_dir / "run_info.json")
                                                                 : (run_dir / "results.json");
    const auto ftime = fs::last_write_time(time_path, ec);
    if (!ec) {
      out.sort_time = to_system_clock(ftime);
      out.timestamp = format_timestamp(out.sort_time);
    }
  }

  if (out.sort_time.time_since_epoch().count() == 0) {
    std::error_code ec;
    const auto ftime = fs::last_write_time(run_dir, ec);
    out.sort_time = ec ? std::chrono::system_clock::now() : to_system_clock(ftime);
    if (out.timestamp.empty()) {
      out.timestamp = format_timestamp(out.sort_time);
    }
  }

  if (!out.snapshot_manifest_path.empty()) {
    fs::path manifest_path = out.snapshot_manifest_path;
    if (!manifest_path.is_absolute()) {
      manifest_path = run_dir / manifest_path;
    }
    std::string snap_err;
    load_snapshot_manifest(manifest_path, out.snapshots, snap_err);
  }
  return true;
}

std::vector<RunManifest> discover_runs(const fs::path& root, size_t max_runs, std::string& error) {
  error.clear();
  std::vector<RunManifest> runs;
  const fs::path runs_dir = root / "build" / "ai_runs";
  if (!fs::exists(runs_dir)) {
    return runs;
  }
  std::error_code ec;
  for (auto it = fs::directory_iterator(runs_dir, ec); it != fs::directory_iterator(); ++it) {
    if (ec) break;
    if (!it->is_directory()) continue;
    RunManifest manifest;
    if (load_run_manifest(it->path(), manifest)) {
      runs.push_back(std::move(manifest));
    }
  }
  std::sort(runs.begin(), runs.end(), [](const RunManifest& a, const RunManifest& b) {
    return a.sort_time > b.sort_time;
  });
  if (max_runs > 0 && runs.size() > max_runs) {
    runs.resize(max_runs);
  }
  return runs;
}

bool load_run_diffs(const RunManifest& run, RunsBrowserState& runs, std::string& error) {
  error.clear();
  runs.diffs.clear();
  runs.diff_selected = 0;
  runs.diffs_loaded = false;
  if (run.staged_patches.empty()) {
    error = "staged patches missing";
    return false;
  }
  const fs::path summary_path = fs::path(run.staged_patches) / "summary.json";
  std::unordered_map<std::string, std::string> diff_targets;
  if (fs::exists(summary_path)) {
    nlohmann::json summary;
    if (load_json_file(summary_path, summary, nullptr)) {
      std::vector<std::string> targets;
      collect_summary_targets(summary, targets, diff_targets);
    }
  }
  std::string diff_err;
  if (!load_diff_entries_from_dir(fs::path(run.staged_patches), diff_targets, runs.diffs, diff_err)) {
    error = diff_err;
    return false;
  }
  runs.diffs_loaded = true;
  runs.diff_is_snapshot = false;
  runs.diff_title = "Staged Diffs";
  return true;
}
#endif

std::vector<std::string> command_lines(const rkg::editor::SubprocessRunner& runner, size_t max_lines) {
  return runner.recent_lines(max_lines);
}

enum class StageStatus { Ok, Fail, Pending, Skipped };

const char* stage_icon(StageStatus status) {
  switch (status) {
    case StageStatus::Ok:
      return "[ok]";
    case StageStatus::Fail:
      return "[fail]";
    case StageStatus::Pending:
      return "[..]";
    case StageStatus::Skipped:
      return "[skip]";
  }
  return "[?]";
}

StageStatus status_for_stage(const AgentRunSummary& run,
                             const fs::path& run_dir,
                             const char* stage_name,
                             bool plan_exists,
                             bool results_exists) {
  if (!run.error_stage.empty() && run.error_stage == stage_name) {
    return StageStatus::Fail;
  }
  if (std::string(stage_name) == "plan") {
    return plan_exists ? StageStatus::Ok : StageStatus::Pending;
  }
  if (std::string(stage_name) == "validate") {
    return plan_exists ? StageStatus::Ok : StageStatus::Pending;
  }
  if (std::string(stage_name) == "stage") {
    return results_exists ? StageStatus::Ok : StageStatus::Pending;
  }
  if (std::string(stage_name) == "apply") {
    if (run.mode != "apply") return StageStatus::Skipped;
    if (!results_exists) return StageStatus::Pending;
    if (run.status == "ok") return StageStatus::Ok;
    return StageStatus::Fail;
  }
  if (std::string(stage_name) == "cook" || std::string(stage_name) == "reload") {
    if (run.mode != "apply") return StageStatus::Skipped;
    if (run.status == "ok") return StageStatus::Pending;
    if (!run.status.empty() && run.status != "ok") return StageStatus::Fail;
    return StageStatus::Pending;
  }
  (void)run_dir;
  return StageStatus::Pending;
}

const char* mesh_id_label(rkg::ecs::MeshId mesh) {
  switch (mesh) {
    case rkg::ecs::MeshId::Cube:
      return "cube";
    case rkg::ecs::MeshId::Quad:
      return "quad";
    case rkg::ecs::MeshId::Unknown:
      return "unknown";
  }
  return "unknown";
}

bool parse_mesh_id_text(const std::string& value, rkg::ecs::MeshId& out) {
  if (value == "cube") {
    out = rkg::ecs::MeshId::Cube;
    return true;
  }
  if (value == "quad") {
    out = rkg::ecs::MeshId::Quad;
    return true;
  }
  return false;
}

std::string entity_display_name(const rkg::runtime::RuntimeHost& runtime, rkg::ecs::Entity entity) {
  for (const auto& pair : runtime.entities_by_name()) {
    if (pair.second == entity) {
      return pair.first;
    }
  }
  return "Entity " + std::to_string(entity);
}

void set_selected_entity(EditorState& state, rkg::ecs::Entity entity) {
  state.selected_entity = entity;
  state.selected_bone = -1;
  state.lock_editor_pivot = false;
  if (entity == rkg::ecs::kInvalidEntity) {
    state.selected_name.clear();
    return;
  }
  state.selected_name = entity_display_name(*state.runtime, entity);
}

rkg::ecs::Registry& registry_mutable(EditorState& state) {
  return const_cast<rkg::ecs::Registry&>(state.runtime->registry());
}

EntitySnapshot snapshot_entity(const rkg::ecs::Registry& registry, rkg::ecs::Entity entity) {
  EntitySnapshot out;
  if (const auto* transform = registry.get_transform(entity)) {
    out.has_transform = true;
    out.transform = *transform;
  }
  if (const auto* renderable = registry.get_renderable(entity)) {
    out.has_renderable = true;
    out.renderable = *renderable;
  }
  return out;
}

bool snapshot_equal(const EntitySnapshot& a, const EntitySnapshot& b, float eps = 0.0005f) {
  if (a.has_transform != b.has_transform) return false;
  if (a.has_renderable != b.has_renderable) return false;
  if (a.has_transform) {
    for (int i = 0; i < 3; ++i) {
      if (std::abs(a.transform.position[i] - b.transform.position[i]) > eps) return false;
      if (std::abs(a.transform.rotation[i] - b.transform.rotation[i]) > eps) return false;
      if (std::abs(a.transform.scale[i] - b.transform.scale[i]) > eps) return false;
    }
  }
  if (a.has_renderable) {
    if (a.renderable.mesh != b.renderable.mesh) return false;
    for (int i = 0; i < 4; ++i) {
      if (std::abs(a.renderable.color[i] - b.renderable.color[i]) > eps) return false;
    }
  }
  return true;
}

void apply_snapshot(rkg::ecs::Registry& registry, rkg::ecs::Entity entity, const EntitySnapshot& snapshot) {
  if (snapshot.has_transform) {
    registry.set_transform(entity, snapshot.transform);
  }
  if (snapshot.has_renderable) {
    registry.set_renderable(entity, snapshot.renderable);
  } else {
    registry.remove_renderable(entity);
  }
}

void mark_overrides_dirty(EditorState& state) {
  state.overrides.dirty = true;
}

void push_undo_entry(UndoState& undo_state,
                     rkg::ecs::Entity entity,
                     const EntitySnapshot& before,
                     const EntitySnapshot& after) {
  if (snapshot_equal(before, after)) {
    return;
  }
  undo_state.undo.push_back({entity, before, after});
  undo_state.redo.clear();
}

void begin_undo_edit(UndoState& undo_state, ImGuiID item_id, rkg::ecs::Entity entity,
                     const rkg::ecs::Registry& registry) {
  if (undo_state.active) return;
  undo_state.active = true;
  undo_state.active_id = item_id;
  undo_state.active_entity = entity;
  undo_state.before = snapshot_entity(registry, entity);
}

void end_undo_edit(UndoState& undo_state, ImGuiID item_id, rkg::ecs::Entity entity,
                   const rkg::ecs::Registry& registry) {
  if (!undo_state.active) return;
  if (undo_state.active_id != item_id || undo_state.active_entity != entity) return;
  const EntitySnapshot after = snapshot_entity(registry, entity);
  push_undo_entry(undo_state, entity, undo_state.before, after);
  undo_state.active = false;
  undo_state.active_id = 0;
  undo_state.active_entity = rkg::ecs::kInvalidEntity;
}

void perform_undo(EditorState& state) {
  if (state.undo.undo.empty()) return;
  const UndoEntry entry = state.undo.undo.back();
  state.undo.undo.pop_back();
  apply_snapshot(registry_mutable(state), entry.entity, entry.before);
  state.undo.redo.push_back(entry);
  mark_overrides_dirty(state);
}

void perform_redo(EditorState& state) {
  if (state.undo.redo.empty()) return;
  const UndoEntry entry = state.undo.redo.back();
  state.undo.redo.pop_back();
  apply_snapshot(registry_mutable(state), entry.entity, entry.after);
  state.undo.undo.push_back(entry);
  mark_overrides_dirty(state);
}

void handle_undo_redo(EditorState& state) {
  ImGuiIO& io = ImGui::GetIO();
  if (state.chat_active || io.WantTextInput) {
    return;
  }
  const bool ctrl = io.KeyCtrl;
  if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Z)) {
    perform_undo(state);
  } else if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Y)) {
    perform_redo(state);
  }
}

bool load_overrides_file(const fs::path& path,
                         std::unordered_map<std::string, EntitySnapshot>& out,
                         std::string& error) {
  out.clear();
#if RKG_ENABLE_DATA_YAML
  if (!fs::exists(path)) {
    return true;
  }
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(path.string());
  } catch (const std::exception& e) {
    error = std::string("overrides load failed: ") + e.what();
    return false;
  }
  if (!doc["overrides"] || !doc["overrides"].IsMap()) {
    return true;
  }
  for (const auto& item : doc["overrides"]) {
    const auto key = item.first.as<std::string>();
    const auto node = item.second;
    EntitySnapshot snapshot{};
    if (node["transform"] && node["transform"].IsMap()) {
      const auto t = node["transform"];
      snapshot.has_transform = true;
      if (t["position"]) {
        auto p = t["position"];
        snapshot.transform.position[0] = p[0].as<float>();
        snapshot.transform.position[1] = p[1].as<float>();
        snapshot.transform.position[2] = p[2].as<float>();
      }
      if (t["rotation"]) {
        auto r = t["rotation"];
        snapshot.transform.rotation[0] = r[0].as<float>();
        snapshot.transform.rotation[1] = r[1].as<float>();
        snapshot.transform.rotation[2] = r[2].as<float>();
      }
      if (t["scale"]) {
        auto s = t["scale"];
        snapshot.transform.scale[0] = s[0].as<float>();
        snapshot.transform.scale[1] = s[1].as<float>();
        snapshot.transform.scale[2] = s[2].as<float>();
      }
    }
    if (node["renderable"] && node["renderable"].IsMap()) {
      const auto r = node["renderable"];
      snapshot.has_renderable = true;
      if (r["mesh"] && r["mesh"].IsScalar()) {
        parse_mesh_id_text(r["mesh"].as<std::string>(), snapshot.renderable.mesh);
      }
      if (r["color"] && r["color"].IsSequence()) {
        const auto c = r["color"];
        snapshot.renderable.color[0] = c[0].as<float>();
        snapshot.renderable.color[1] = c.size() > 1 ? c[1].as<float>() : snapshot.renderable.color[1];
        snapshot.renderable.color[2] = c.size() > 2 ? c[2].as<float>() : snapshot.renderable.color[2];
        snapshot.renderable.color[3] = c.size() > 3 ? c[3].as<float>() : snapshot.renderable.color[3];
      }
    }
    out.emplace(key, snapshot);
  }
  return true;
#else
  (void)path;
  error = "YAML support disabled";
  return false;
#endif
}

bool write_overrides_file(const fs::path& path,
                          const std::unordered_map<std::string, EntitySnapshot>& overrides,
                          std::string& error) {
#if RKG_ENABLE_DATA_YAML
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "overrides" << YAML::Value << YAML::BeginMap;
  for (const auto& entry : overrides) {
    out << YAML::Key << entry.first << YAML::Value;
    out << YAML::BeginMap;
    if (entry.second.has_transform) {
      const auto& t = entry.second.transform;
      out << YAML::Key << "transform" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "position" << YAML::Value << YAML::Flow
          << YAML::BeginSeq << t.position[0] << t.position[1] << t.position[2] << YAML::EndSeq;
      out << YAML::Key << "rotation" << YAML::Value << YAML::Flow
          << YAML::BeginSeq << t.rotation[0] << t.rotation[1] << t.rotation[2] << YAML::EndSeq;
      out << YAML::Key << "scale" << YAML::Value << YAML::Flow
          << YAML::BeginSeq << t.scale[0] << t.scale[1] << t.scale[2] << YAML::EndSeq;
      out << YAML::EndMap;
    }
    if (entry.second.has_renderable) {
      const auto& r = entry.second.renderable;
      out << YAML::Key << "renderable" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "mesh" << YAML::Value << mesh_id_label(r.mesh);
      out << YAML::Key << "color" << YAML::Value << YAML::Flow
          << YAML::BeginSeq << r.color[0] << r.color[1] << r.color[2] << r.color[3] << YAML::EndSeq;
      out << YAML::EndMap;
    }
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
  out << YAML::EndMap;

  std::ofstream file(path);
  if (!file) {
    error = "failed to write overrides file";
    return false;
  }
  file << out.c_str() << "\n";
  return true;
#else
  (void)path;
  (void)overrides;
  error = "YAML support disabled";
  return false;
#endif
}

std::unordered_map<std::string, EntitySnapshot> build_override_map(const rkg::runtime::RuntimeHost& runtime) {
  std::unordered_map<std::string, EntitySnapshot> out;
  const auto& registry = runtime.registry();
  for (const auto& entry : runtime.entity_override_keys()) {
    const auto entity = entry.first;
    const auto& key = entry.second;
    EntitySnapshot snapshot = snapshot_entity(registry, entity);
    if (!snapshot.has_transform && !snapshot.has_renderable) {
      continue;
    }
    out.emplace(key, snapshot);
  }
  return out;
}

size_t count_dirty_overrides(const EditorState& state, size_t* total_out = nullptr) {
  if (!state.runtime) return 0;
  const auto current = build_override_map(*state.runtime);
  if (total_out) {
    *total_out = current.size();
  }
  size_t dirty = 0;
  for (const auto& kv : current) {
    const auto it = state.overrides.saved.find(kv.first);
    if (it == state.overrides.saved.end() || !snapshot_equal(kv.second, it->second)) {
      ++dirty;
    }
  }
  for (const auto& kv : state.overrides.saved) {
    if (current.find(kv.first) == current.end()) {
      ++dirty;
    }
  }
  return dirty;
}

void sync_overrides_state(EditorState& state) {
#if RKG_ENABLE_DATA_YAML
  if (!state.runtime) return;
  if (state.overrides.path.empty()) {
    state.overrides.path = state.runtime->project_root() / "editor_overrides.yaml";
  }
  const std::string reload_time = state.runtime->last_reload_time();
  if (reload_time != state.overrides.last_reload_time) {
    state.overrides.last_reload_time = reload_time;
    state.overrides.last_error.clear();
    load_overrides_file(state.overrides.path, state.overrides.saved, state.overrides.last_error);
    state.overrides.dirty = false;
    state.undo.undo.clear();
    state.undo.redo.clear();
    state.undo.active = false;
    state.selected_entity = rkg::ecs::kInvalidEntity;
    state.selected_name.clear();
    state.auto_select = true;
  }
#else
  (void)state;
#endif
}

bool save_overrides(EditorState& state) {
#if RKG_ENABLE_DATA_YAML
  state.overrides.last_error.clear();
  const auto overrides = build_override_map(*state.runtime);
  if (!write_overrides_file(state.overrides.path, overrides, state.overrides.last_error)) {
    return false;
  }
  state.overrides.saved = overrides;
  state.overrides.dirty = false;
  return true;
#else
  state.overrides.last_error = "Overrides require YAML support";
  return false;
#endif
}

bool revert_selected(EditorState& state) {
#if RKG_ENABLE_DATA_YAML
  state.overrides.last_error.clear();
  if (state.selected_entity == rkg::ecs::kInvalidEntity) {
    state.overrides.last_error = "no entity selected";
    return false;
  }
  const std::string key = state.runtime->override_key_for_entity(state.selected_entity);
  if (key.empty()) {
    state.overrides.last_error = "selected entity has no override id";
    return false;
  }
  std::unordered_map<std::string, EntitySnapshot> overrides = state.overrides.saved;
  if (!load_overrides_file(state.overrides.path, overrides, state.overrides.last_error) &&
      !state.overrides.last_error.empty()) {
    return false;
  }
  overrides.erase(key);
  if (!write_overrides_file(state.overrides.path, overrides, state.overrides.last_error)) {
    return false;
  }
  state.overrides.saved = overrides;
  state.overrides.dirty = false;
  state.runtime->force_reload("editor override revert selected");
  state.selected_entity = rkg::ecs::kInvalidEntity;
  state.selected_name.clear();
  state.auto_select = true;
  state.fallback_entity = rkg::ecs::kInvalidEntity;
  return true;
#else
  state.overrides.last_error = "Overrides require YAML support";
  return false;
#endif
}

bool revert_all(EditorState& state) {
#if RKG_ENABLE_DATA_YAML
  state.overrides.last_error.clear();
  if (fs::exists(state.overrides.path)) {
    std::error_code ec;
    fs::remove(state.overrides.path, ec);
    if (ec) {
      state.overrides.last_error = "failed to remove overrides file";
      return false;
    }
  }
  state.overrides.saved.clear();
  state.overrides.dirty = false;
  state.runtime->force_reload("editor override revert all");
  state.selected_entity = rkg::ecs::kInvalidEntity;
  state.selected_name.clear();
  state.auto_select = true;
  state.fallback_entity = rkg::ecs::kInvalidEntity;
  return true;
#else
  state.overrides.last_error = "Overrides require YAML support";
  return false;
#endif
}

std::filesystem::path rkgctl_path(const rkg::runtime::RuntimeHost& runtime) {
#if defined(_WIN32)
  return runtime.executable_dir() / "rkgctl.exe";
#else
  return runtime.executable_dir() / "rkgctl";
#endif
}

bool start_status(EditorState& state) {
  auto& agent = state.agent;
  const auto cmd = rkgctl_path(*state.runtime);
  if (!fs::exists(cmd)) {
    agent.last_error = "rkgctl not found: " + cmd.string();
    agent.stage = AgentPanelState::Stage::Idle;
    return false;
  }

  const auto project_path = state.runtime->project_root();
  std::vector<std::string> args = {cmd.string(), "agent", "status", "--project",
                                   project_path.string(), "--runs", "1", "--json"};
  if (!agent.status_process.start(args, state.runtime->paths().root)) {
    agent.last_error = agent.status_process.error_message();
    agent.stage = AgentPanelState::Stage::Idle;
    return false;
  }
  agent.last_command_label = "agent status";
  return true;
}

bool start_agent_plan(EditorState& state) {
  auto& agent = state.agent;
  agent.last_error.clear();
  agent.plan_summary = {};
  agent.plan_path.clear();
  agent.plan_path_buffer[0] = '\0';

  const auto cmd = rkgctl_path(*state.runtime);
  if (!fs::exists(cmd)) {
    agent.last_error = "rkgctl not found: " + cmd.string();
    return false;
  }
  const auto project_path = state.runtime->project_root();
  std::vector<std::string> args = {cmd.string(), "agent", "openai", "plan", "--project",
                                   project_path.string(), "--goal", agent.goal};

  if (!agent.agent_process.start(args, state.runtime->paths().root)) {
    agent.last_error = agent.agent_process.error_message();
    return false;
  }
  agent.stage = AgentPanelState::Stage::RunningAgent;
  agent.last_command_label = "agent openai plan";
  return true;
}

bool start_agent_apply(EditorState& state, const std::string& plan_path) {
  auto& agent = state.agent;
  agent.last_error.clear();

  const auto cmd = rkgctl_path(*state.runtime);
  if (!fs::exists(cmd)) {
    agent.last_error = "rkgctl not found: " + cmd.string();
    return false;
  }
  if (plan_path.empty()) {
    agent.last_error = "plan path missing";
    return false;
  }
  const auto project_path = state.runtime->project_root();
  std::vector<std::string> args = {cmd.string(), "agent", "offline", "apply", "--project",
                                   project_path.string(), "--in", plan_path, "--yes"};
  if (!agent.agent_process.start(args, state.runtime->paths().root)) {
    agent.last_error = agent.agent_process.error_message();
    return false;
  }
  agent.stage = AgentPanelState::Stage::RunningAgent;
  agent.last_command_label = "agent offline apply";
  return true;
}

void update_agent_state(EditorState& state) {
  auto& agent = state.agent;
  agent.agent_process.update();
  agent.status_process.update();
  agent.open_process.update();

  if (agent.stage == AgentPanelState::Stage::RunningAgent && agent.agent_process.finished()) {
    agent.stage = AgentPanelState::Stage::RunningStatus;
    start_status(state);
  } else if (agent.stage == AgentPanelState::Stage::RunningStatus && agent.status_process.finished()) {
#if RKG_ENABLE_DATA_JSON
    const auto lines = agent.status_process.recent_lines(200);
    const std::string payload = join_lines(lines);
    agent.status = parse_agent_status_json(payload, state.runtime->paths().root);
    if (agent.status.valid && agent.status.last_run.valid) {
      if (agent.status.last_run.plan_path.empty() && !agent.status.last_run.run_dir.empty()) {
        agent.status.last_run.plan_path = (fs::path(agent.status.last_run.run_dir) / "plan.json").generic_string();
      }
      if (!agent.status.last_run.plan_path.empty()) {
        agent.plan_path = agent.status.last_run.plan_path;
        std::snprintf(agent.plan_path_buffer, sizeof(agent.plan_path_buffer), "%s", agent.plan_path.c_str());
        agent.plan_summary = load_plan_summary(agent.status.last_run.plan_path);
      }
    }
#else
    agent.last_error = "JSON disabled; agent status unavailable";
#endif
    agent.stage = AgentPanelState::Stage::Idle;
  }
}

bool start_cook(ContentPanelState& content, const rkg::runtime::RuntimeHost& runtime) {
  const auto cmd = rkgctl_path(runtime);
  if (!fs::exists(cmd)) {
    content.last_error = "rkgctl not found: " + cmd.string();
    return false;
  }
  std::vector<std::string> args = {cmd.string(), "content", "cook", "--project", runtime.project_root().string()};
  if (!content.cook_process.start(args, runtime.paths().root)) {
    content.last_error = content.cook_process.error_message();
    return false;
  }
  return true;
}

bool start_watch(ContentPanelState& content, const rkg::runtime::RuntimeHost& runtime) {
  const auto cmd = rkgctl_path(runtime);
  if (!fs::exists(cmd)) {
    content.last_error = "rkgctl not found: " + cmd.string();
    return false;
  }
  std::vector<std::string> args = {cmd.string(), "content", "watch", "--project", runtime.project_root().string()};
  if (!content.watch_process.start(args, runtime.paths().root)) {
    content.last_error = content.watch_process.error_message();
    return false;
  }
  return true;
}

void reset_commit_state(ContentPanelState& content) {
  content.commit_error.clear();
  content.commit_run_id.clear();
  content.commit_run_dir.clear();
  content.commit_staging_dir.clear();
  content.commit_results_path.clear();
  content.commit_status.clear();
  content.commit_stage.clear();
  content.commit_error_code.clear();
  content.commit_error_message.clear();
  content.commit_entity_id.clear();
  content.commit_selector_type.clear();
  content.commit_selector_value.clear();
  content.commit_selector_warning.clear();
  content.commit_conflicts = 0;
  content.commit_conflict_files.clear();
  content.commit_conflict_detected = false;
  content.commit_forced_apply = false;
  content.commit_snapshots_taken = false;
  content.commit_snapshot_manifest.clear();
  content.commit_dry_run = true;
  content.commit_last_exit = -1;
  content.commit_force_modal = false;
}

void reset_diff_preview(DiffPreviewState& preview) {
  preview.run_id.clear();
  preview.run_dir.clear();
  preview.level_path.clear();
  preview.entity_id.clear();
  preview.selector_type.clear();
  preview.selector_value.clear();
  preview.selector_warning.clear();
  preview.diffs.clear();
  preview.selected = 0;
  preview.loaded = false;
  preview.error.clear();
}

std::string current_level_arg(const rkg::runtime::RuntimeHost& runtime) {
  if (runtime.current_level_path().empty()) {
    return {};
  }
  fs::path level = runtime.current_level_path();
  if (level.is_absolute()) {
    std::error_code ec;
    level = fs::relative(level, runtime.project_root(), ec);
    if (ec) {
      return runtime.current_level_path();
    }
  }
  return level.generic_string();
}

bool start_commit_stage(ContentPanelState& content,
                        const rkg::runtime::RuntimeHost& runtime,
                        const std::string& entity_id,
                        const std::string& entity_name) {
  const auto cmd = rkgctl_path(runtime);
  if (!fs::exists(cmd)) {
    content.commit_error = "rkgctl not found: " + cmd.string();
    return false;
  }
  content.commit_process.clear_output();
  std::vector<std::string> args = {cmd.string(),
                                   "content",
                                   "commit-overrides",
                                   "--project",
                                   runtime.project_root().string(),
                                   "--overrides",
                                   (runtime.project_root() / "editor_overrides.yaml").string(),
                                   "--stage"};
  const std::string level_arg = current_level_arg(runtime);
  if (!level_arg.empty()) {
    args.push_back("--level");
    args.push_back(level_arg);
  }
  if (!entity_id.empty()) {
    args.push_back("--entity-id");
    args.push_back(entity_id);
  } else if (!entity_name.empty()) {
    args.push_back("--entity-name");
    args.push_back(entity_name);
  }
  reset_commit_state(content);
  content.commit_dry_run = true;
  content.commit_error.clear();
  if (!content.commit_process.start(args, runtime.paths().root)) {
    content.commit_error = content.commit_process.error_message();
    return false;
  }
  return true;
}

bool start_commit_apply_staged(ContentPanelState& content,
                               const rkg::runtime::RuntimeHost& runtime,
                               const std::string& run_dir,
                               bool force) {
  const auto cmd = rkgctl_path(runtime);
  if (!fs::exists(cmd)) {
    content.commit_error = "rkgctl not found: " + cmd.string();
    return false;
  }
  if (run_dir.empty()) {
    content.commit_error = "staged run dir missing";
    return false;
  }
  content.commit_process.clear_output();
  std::vector<std::string> args = {cmd.string(),
                                   "content",
                                   "commit-overrides",
                                   "--project",
                                   runtime.project_root().string(),
                                   "--apply-staged",
                                   run_dir,
                                   "--yes"};
  if (force) {
    args.push_back("--force");
  }
  reset_commit_state(content);
  content.commit_dry_run = false;
  content.commit_run_dir = run_dir;
  content.commit_error.clear();
  if (!content.commit_process.start(args, runtime.paths().root)) {
    content.commit_error = content.commit_process.error_message();
    return false;
  }
  return true;
}

bool start_open_folder(AgentPanelState& agent, const std::string& run_dir, const rkg::runtime::RuntimeHost& runtime) {
  agent.open_error.clear();
  if (run_dir.empty()) {
    agent.open_error = "run dir missing";
    return false;
  }
#if defined(_WIN32)
  (void)runtime;
  agent.open_error = "open folder not implemented on Windows";
  return false;
#else
  std::vector<std::string> args = {"xdg-open", run_dir};
  if (!agent.open_process.start(args, runtime.paths().root)) {
    agent.open_error = agent.open_process.error_message();
    return false;
  }
  return true;
#endif
}

bool start_open_file(ContentPanelState& content,
                     const std::string& path,
                     const rkg::runtime::RuntimeHost& runtime) {
  content.open_file_error.clear();
  if (path.empty()) {
    content.open_file_error = "file path missing";
    return false;
  }
#if defined(_WIN32)
  (void)runtime;
  content.open_file_error = "open file not implemented on Windows";
  return false;
#else
  std::vector<std::string> args = {"xdg-open", path};
  if (!content.open_file_process.start(args, runtime.paths().root)) {
    content.open_file_error = content.open_file_process.error_message();
    return false;
  }
  return true;
#endif
}

bool start_open_folder(RunsBrowserState& runs,
                       const std::string& path,
                       const rkg::runtime::RuntimeHost& runtime) {
  runs.open_error.clear();
  if (path.empty()) {
    runs.open_error = "run dir missing";
    return false;
  }
#if defined(_WIN32)
  (void)runtime;
  runs.open_error = "open folder not implemented on Windows";
  return false;
#else
  std::vector<std::string> args = {"xdg-open", path};
  if (!runs.open_process.start(args, runtime.paths().root)) {
    runs.open_error = runs.open_process.error_message();
    return false;
  }
  return true;
#endif
}

bool start_open_file(RunsBrowserState& runs,
                     const std::string& path,
                     const rkg::runtime::RuntimeHost& runtime) {
  runs.open_file_error.clear();
  if (path.empty()) {
    runs.open_file_error = "file path missing";
    return false;
  }
#if defined(_WIN32)
  (void)runtime;
  runs.open_file_error = "open file not implemented on Windows";
  return false;
#else
  std::vector<std::string> args = {"xdg-open", path};
  if (!runs.open_file_process.start(args, runtime.paths().root)) {
    runs.open_file_error = runs.open_file_process.error_message();
    return false;
  }
  return true;
#endif
}

bool stage_snapshot_restore_run(RunsBrowserState& runs,
                                const SnapshotEntry& entry,
                                const RunManifest& run,
                                const rkg::runtime::RuntimeHost& runtime) {
  runs.restore_error.clear();
  const fs::path root = runtime.paths().root;
  const fs::path target = entry.path;
  const fs::path snapshot = entry.snapshot_path;
  const auto result = rkg::stage_snapshot_restore(root, target, snapshot, run.run_id);
  if (!result.success) {
    runs.restore_error = result.error_message.empty() ? "snapshot stage failed" : result.error_message;
    return false;
  }
  runs.selected_run_dir = result.run_dir.generic_string();
  return true;
}

bool apply_snapshot_restore_run(RunsBrowserState& runs,
                                const RunManifest& run,
                                const rkg::runtime::RuntimeHost& runtime) {
  (void)runtime;
  const auto result = rkg::apply_snapshot_restore(run.run_dir, false);
  if (!result.success && result.error_message.empty()) {
    runs.restore_error = "snapshot apply failed";
    return false;
  }
  return true;
}

#if RKG_ENABLE_DATA_JSON
void autoload_latest_staged(EditorState& state) {
  if (state.diff_autoload_done || !state.runtime) {
    return;
  }
  state.diff_autoload_done = true;
  std::string err;
  const auto run_dir = rkg::find_latest_staged_run_dir(state.runtime->paths().root, &err);
  if (run_dir.has_value()) {
    std::string diff_err;
    if (!load_commit_diff_preview(run_dir.value(), state.diff_preview, diff_err)) {
      state.diff_preview.error = diff_err;
    }
  } else if (!err.empty()) {
    state.diff_preview.error = err;
  }
}

std::string truncate_text(const std::string& text, size_t max_len) {
  if (text.size() <= max_len) return text;
  if (max_len < 4) return text.substr(0, max_len);
  return text.substr(0, max_len - 3) + "...";
}

const char* run_status_icon(const RunManifest& run) {
  if (run.incomplete) return "[..]";
  if (!run.error_code.empty()) return "[fail]";
  if (!run.status.empty() && run.status != "ok") return "[fail]";
  if (run.success) return "[ok]";
  return "[..]";
}

std::string to_lower_copy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

bool contains_case_insensitive(const std::string& haystack, const std::string& needle) {
  if (needle.empty()) return true;
  const std::string h = to_lower_copy(haystack);
  const std::string n = to_lower_copy(needle);
  return h.find(n) != std::string::npos;
}

enum class RunStatusKind { Success, Failed, Conflict, Unknown };

RunStatusKind run_status_kind(const RunManifest& run) {
  if (run.error_code == "conflict" || run.status == "conflicts" || !run.conflict_files.empty()) {
    return RunStatusKind::Conflict;
  }
  if (run.success || run.status == "ok") {
    return RunStatusKind::Success;
  }
  if (!run.error_code.empty() || run.status == "failed") {
    return RunStatusKind::Failed;
  }
  return RunStatusKind::Unknown;
}

bool run_is_ai(const RunManifest& run) {
  return run.run_type == "agent_plan" || run.run_type == "agent_apply" || run.run_type == "offline_apply";
}

bool run_is_commit(const RunManifest& run) {
  return run.run_type == "commit_overrides_stage" || run.run_type == "commit_overrides_apply";
}

bool run_is_snapshot(const RunManifest& run) {
  return run.run_type == "snapshot_restore_stage" || run.run_type == "snapshot_restore_apply";
}

bool run_matches_filters(const RunManifest& run, const RunsBrowserState& state) {
  if (run_is_ai(run) && !state.filter_ai) return false;
  if (run_is_commit(run) && !state.filter_commit) return false;
  if (run_is_snapshot(run) && !state.filter_snapshot) return false;
  if (!run_is_ai(run) && !run_is_commit(run) && !run_is_snapshot(run) && !state.filter_other) return false;

  const RunStatusKind status = run_status_kind(run);
  if (status == RunStatusKind::Success && !state.status_success) return false;
  if (status == RunStatusKind::Failed && !state.status_failed) return false;
  if (status == RunStatusKind::Conflict && !state.status_conflict) return false;
  if (status == RunStatusKind::Unknown && !state.status_unknown) return false;

  const std::string needle = state.search_buffer;
  if (needle.empty()) return true;
  if (contains_case_insensitive(run.run_id, needle)) return true;
  if (contains_case_insensitive(run.goal, needle)) return true;
  if (contains_case_insensitive(run.run_type, needle)) return true;
  if (contains_case_insensitive(run.run_dir, needle)) return true;
  if (contains_case_insensitive(run.snapshot_run_id, needle)) return true;
  if (contains_case_insensitive(run.snapshot_path, needle)) return true;
  if (contains_case_insensitive(run.staged_patches, needle)) return true;
  if (contains_case_insensitive(run.snapshot_manifest_path, needle)) return true;
  for (const auto& path : run.target_files) {
    if (contains_case_insensitive(path, needle)) return true;
  }
  for (const auto& path : run.conflict_files) {
    if (contains_case_insensitive(path, needle)) return true;
  }
  return false;
}

void refresh_runs_browser(EditorState& state, bool force) {
  if (!state.runtime) return;
  auto& runs = state.runs;
  const auto now = std::chrono::steady_clock::now();
  if (!force && runs.loaded &&
      std::chrono::duration_cast<std::chrono::milliseconds>(now - runs.last_refresh).count() < 1500) {
    return;
  }
  runs.last_refresh = now;
  std::string err;
  const std::string selected_dir = runs.selected_run_dir;
  runs.runs = discover_runs(state.runtime->paths().root, 50, err);
  runs.loaded = true;
  runs.runs_error = err;
  for (auto& run : runs.runs) {
    if (run.project_path.empty()) {
      run.project_path = state.runtime->project_root().string();
    }
  }

  if (!selected_dir.empty()) {
    for (size_t i = 0; i < runs.runs.size(); ++i) {
      if (runs.runs[i].run_dir == selected_dir) {
        runs.selected = static_cast<int>(i);
        break;
      }
    }
  }
  if (runs.selected < 0 || runs.selected >= static_cast<int>(runs.runs.size())) {
    runs.selected = runs.runs.empty() ? 0 : 0;
  }
}

bool start_restage_commit(RunsBrowserState& runs,
                          const RunManifest& manifest,
                          const rkg::runtime::RuntimeHost& runtime) {
  const fs::path cmd = rkgctl_path(runtime);
  if (!fs::exists(cmd)) {
    runs.restage_error = "rkgctl not found: " + cmd.generic_string();
    return false;
  }
  if (manifest.overrides_path.empty()) {
    runs.restage_error = "overrides path missing";
    return false;
  }
  runs.restage_error.clear();
  runs.restage_output_error.clear();
  runs.restage_output_run_dir.clear();
  runs.restage_pending = true;
  runs.restage_process.clear_output();
  std::vector<std::string> args = {cmd.string(),
                                   "content",
                                   "commit-overrides",
                                   "--project",
                                   runtime.project_root().string(),
                                   "--overrides",
                                   manifest.overrides_path,
                                   "--stage"};
  if (!manifest.level_path.empty()) {
    args.push_back("--level");
    args.push_back(manifest.level_path);
  }
  if (manifest.selector_type == "id" && !manifest.selector_value.empty()) {
    args.push_back("--entity-id");
    args.push_back(manifest.selector_value);
  } else if (manifest.selector_type == "name" && !manifest.selector_value.empty()) {
    args.push_back("--entity-name");
    args.push_back(manifest.selector_value);
  } else if (!manifest.entity_id.empty()) {
    args.push_back("--entity-id");
    args.push_back(manifest.entity_id);
  } else if (!manifest.entity_name.empty()) {
    args.push_back("--entity-name");
    args.push_back(manifest.entity_name);
  }
  if (!runs.restage_process.start(args, runtime.paths().root)) {
    runs.restage_error = runs.restage_process.error_message();
    runs.restage_pending = false;
    return false;
  }
  return true;
}

void update_runs_browser_state(EditorState& state) {
  auto& runs = state.runs;
  runs.restage_process.update();
  runs.open_process.update();
  runs.open_file_process.update();

  if (runs.restage_pending && runs.restage_process.finished()) {
    runs.restage_pending = false;
    if (runs.restage_process.exit_code() != 0) {
      runs.restage_output_error = "restage failed";
    }
    refresh_runs_browser(state, true);
    std::string err;
    const auto run_dir = rkg::find_latest_staged_run_dir(state.runtime->paths().root, &err);
    if (run_dir.has_value()) {
      runs.selected_run_dir = run_dir.value();
      for (size_t i = 0; i < runs.runs.size(); ++i) {
        if (runs.runs[i].run_dir == runs.selected_run_dir) {
          runs.selected = static_cast<int>(i);
          runs.diffs_loaded = false;
          runs.diff_error.clear();
          runs.diff_title.clear();
          runs.diff_is_snapshot = false;
          runs.diffs.clear();
          break;
        }
      }
    } else if (!err.empty()) {
      runs.restage_output_error = err;
    }
  }

  refresh_runs_browser(state, false);
}
#else
void autoload_latest_staged(EditorState& state) {
  (void)state;
}

void update_runs_browser_state(EditorState& state) {
  (void)state;
}
#endif

void update_content_state(EditorState& state) {
  state.content.cook_process.update();
  state.content.watch_process.update();
  state.content.commit_process.update();
  state.content.open_file_process.update();
  if (state.content.commit_process.finished() &&
      state.content.commit_last_exit != state.content.commit_process.exit_code()) {
    state.content.commit_last_exit = state.content.commit_process.exit_code();
#if RKG_ENABLE_DATA_JSON
    std::string err;
    if (!load_commit_overrides_summary(state.runtime->paths().root, state.content, err)) {
      state.content.commit_error = err;
    } else {
      if (!state.content.commit_run_dir.empty() &&
          (state.diff_preview.run_dir != state.content.commit_run_dir ||
           state.diff_preview.run_id != state.content.commit_run_id)) {
        std::string diff_err;
        if (!load_commit_diff_preview(state.content.commit_run_dir, state.diff_preview, diff_err)) {
          state.diff_preview.error = diff_err;
        }
      }
      if (!state.content.commit_dry_run &&
          state.content.commit_stage == "apply" &&
          state.content.commit_status == "ok" &&
          state.content.commit_run_id != state.content.commit_handled_run_id) {
        state.content.commit_handled_run_id = state.content.commit_run_id;
        state.content.commit_clear_pending = true;
      }
    }
#else
    state.content.commit_error = "JSON disabled; commit summary unavailable";
#endif
  }

  autoload_latest_staged(state);

  if (state.content.commit_clear_pending) {
    state.content.commit_clear_pending = false;
    revert_all(state);
  }
}

#if RKG_EDITOR_IMGUI
void draw_copy_button(const char* id, const std::string& value) {
  ImGui::SameLine();
  if (ImGui::SmallButton(id)) {
    ImGui::SetClipboardText(value.c_str());
  }
}

bool draw_diff_entries_viewer(std::vector<DiffPreviewEntry>& entries,
                              int& selected,
                              const char* list_id,
                              const char* text_id) {
  if (entries.empty()) {
    ImGui::TextUnformatted("No diff files found.");
    return false;
  }

  selected = std::clamp(selected, 0, static_cast<int>(entries.size()) - 1);
  ImGui::BeginChild(list_id, ImVec2(220, 0), true);
  for (int i = 0; i < static_cast<int>(entries.size()); ++i) {
    const auto& entry = entries[i];
    const std::string label = entry.target_path.empty() ? entry.diff_path : entry.target_path;
    if (ImGui::Selectable(label.c_str(), selected == i)) {
      selected = i;
    }
  }
  ImGui::EndChild();
  ImGui::SameLine();
  ImGui::BeginChild(text_id, ImVec2(0, 0), true, ImGuiWindowFlags_HorizontalScrollbar);
  const auto& entry = entries[selected];
  ImGui::TextUnformatted(entry.diff_text.empty() ? "(diff empty)" : entry.diff_text.c_str());
  ImGui::EndChild();
  return entry.truncated;
}

void build_dock_layout(EditorState& state) {
  ImGuiID dockspace_id = ImGui::GetID("EditorDockspace");
  ImGui::DockBuilderRemoveNode(dockspace_id);
  ImGuiDockNodeFlags dock_flags =
      static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_DockSpace) |
      static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_PassthruCentralNode);
  ImGui::DockBuilderAddNode(dockspace_id, dock_flags);
  ImGui::DockBuilderSetNodeSize(dockspace_id, ImGui::GetMainViewport()->Size);

  ImGuiID dock_main = dockspace_id;
  ImGuiID dock_left = ImGui::DockBuilderSplitNode(dock_main, ImGuiDir_Left, 0.22f, nullptr, &dock_main);
  ImGuiID dock_right = ImGui::DockBuilderSplitNode(dock_main, ImGuiDir_Right, 0.28f, nullptr, &dock_main);
  ImGuiID dock_bottom = ImGui::DockBuilderSplitNode(dock_main, ImGuiDir_Down, 0.25f, nullptr, &dock_main);
  ImGuiID dock_top = ImGui::DockBuilderSplitNode(dock_main, ImGuiDir_Up, 0.08f, nullptr, &dock_main);

  ImGuiID dock_left_bottom = ImGui::DockBuilderSplitNode(dock_left, ImGuiDir_Down, 0.5f, nullptr, &dock_left);
  ImGuiID dock_right_bottom = ImGui::DockBuilderSplitNode(dock_right, ImGuiDir_Down, 0.4f, nullptr, &dock_right);
  ImGuiID dock_bottom_right = ImGui::DockBuilderSplitNode(dock_bottom, ImGuiDir_Right, 0.45f, nullptr, &dock_bottom);

  ImGui::DockBuilderDockWindow("Viewport", dock_main);
  ImGui::DockBuilderDockWindow("Toolbar", dock_top);
  ImGui::DockBuilderDockWindow("Scene", dock_left);
  ImGui::DockBuilderDockWindow("Inspector", dock_left_bottom);
  ImGui::DockBuilderDockWindow("Content", dock_bottom);
  ImGui::DockBuilderDockWindow("Diff Preview", dock_bottom_right);
  ImGui::DockBuilderDockWindow("Chat", dock_right);
  ImGui::DockBuilderDockWindow("Runs", dock_right_bottom);

  ImGui::DockBuilderFinish(dockspace_id);
  state.dock_built = true;
}

void draw_toolbar(EditorState& state) {
  ImGui::Begin("Toolbar", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar |
                                 ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse);

  const bool playing = state.play_state == PlayState::Play;
  const bool paused = state.play_state == PlayState::Pause;
  const char* label = playing ? "PLAY" : (paused ? "PAUSED" : "EDIT");
  const float dt = ImGui::GetIO().DeltaTime;
  const float fps = dt > 0.0f ? (1.0f / dt) : 0.0f;

  if (ImGui::Button("Play")) {
    if (state.play_state != PlayState::Play) {
      state.saved_editor_cam.valid = true;
      state.saved_editor_cam.yaw = state.camera_yaw;
      state.saved_editor_cam.pitch = state.camera_pitch;
      state.saved_editor_cam.distance = state.camera_distance;
      state.saved_editor_cam.pan[0] = state.camera_pan[0];
      state.saved_editor_cam.pan[1] = state.camera_pan[1];
      state.saved_editor_cam.pan[2] = state.camera_pan[2];
      state.saved_editor_cam.pivot[0] = state.editor_pivot_world[0];
      state.saved_editor_cam.pivot[1] = state.editor_pivot_world[1];
      state.saved_editor_cam.pivot[2] = state.editor_pivot_world[2];
      state.camera_pan[0] = 0.0f;
      state.camera_pan[1] = 0.0f;
      state.camera_pan[2] = 0.0f;
      state.lock_editor_pivot = false;
    }
    state.play_state = PlayState::Play;
  }
  ImGui::SameLine();
  if (ImGui::Button("Pause")) {
    state.play_state = PlayState::Pause;
  }
  ImGui::SameLine();
  if (ImGui::Button("Step")) {
    state.step_requested = true;
    state.play_state = PlayState::Pause;
  }
  ImGui::SameLine();
  if (ImGui::Button("Stop")) {
    state.play_state = PlayState::Edit;
    state.stop_requested = true;
    if (state.saved_editor_cam.valid) {
      state.camera_yaw = state.saved_editor_cam.yaw;
      state.camera_pitch = state.saved_editor_cam.pitch;
      state.camera_distance = state.saved_editor_cam.distance;
      state.camera_pan[0] = state.saved_editor_cam.pan[0];
      state.camera_pan[1] = state.saved_editor_cam.pan[1];
      state.camera_pan[2] = state.saved_editor_cam.pan[2];
      state.editor_pivot_world[0] = state.saved_editor_cam.pivot[0];
      state.editor_pivot_world[1] = state.saved_editor_cam.pivot[1];
      state.editor_pivot_world[2] = state.saved_editor_cam.pivot[2];
      state.lock_editor_pivot = true;
    }
  }

  ImGui::SameLine();
  ImGui::Text("Mode: %s | dt %.2f ms | %.1f fps", label, dt * 1000.0f, fps);

  ImGui::End();
}

void draw_viewport(EditorState& state) {
  ImGui::Begin("Viewport", nullptr,
               ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
                   ImGuiWindowFlags_NoCollapse);
  if (ImGui::CollapsingHeader("Grid/Debug", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("World Grid", &state.show_world_grid);
    ImGui::SameLine();
    ImGui::Checkbox("Character Grid", &state.show_character_grid);
    ImGui::SameLine();
    ImGui::Checkbox("World Axes", &state.show_world_axes);
    ImGui::SameLine();
    ImGui::Checkbox("Face Labels", &state.show_face_labels);
    ImGui::SameLine();
    ImGui::Checkbox("Skeletons", &state.show_skeleton_debug);
    ImGui::SliderFloat("Grid Half Extent", &state.grid_half_extent, 1.0f, 50.0f, "%.1f");
    ImGui::SliderFloat("Grid Step", &state.grid_step, 0.25f, 5.0f, "%.2f");
    if (state.grid_step < 0.25f) state.grid_step = 0.25f;
    if (state.grid_half_extent < state.grid_step) {
      state.grid_half_extent = state.grid_step;
    }
    ImGui::Separator();
  }
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const ImVec2 cursor = ImGui::GetCursorScreenPos();
  state.viewport_pos[0] = cursor.x;
  state.viewport_pos[1] = cursor.y;
  state.viewport_size[0] = avail.x;
  state.viewport_size[1] = avail.y;
  const int width = static_cast<int>(avail.x);
  const int height = static_cast<int>(avail.y);
#if RKG_EDITOR_IMGUI
  rkg::debug_ui::set_viewport_size(width > 0 ? width : 0, height > 0 ? height : 0);
  void* viewport_tex = rkg::debug_ui::viewport_texture();
  if (viewport_tex && rkg::debug_ui::viewport_supported()) {
    ImGui::Image(viewport_tex, avail);
  } else {
    const char* err = rkg::debug_ui::viewport_error();
    ImGui::TextUnformatted("Render-to-texture not supported on this renderer yet.");
    if (err) {
      ImGui::TextWrapped("Detail: %s", err);
    }
  }
#else
  ImGui::TextUnformatted("Render-to-texture not supported on this build.");
#endif

  const ImGuiIO& io = ImGui::GetIO();
  const bool hovered = ImGui::IsWindowHovered();
  const bool clicked_any =
      ImGui::IsMouseClicked(ImGuiMouseButton_Left) ||
      ImGui::IsMouseClicked(ImGuiMouseButton_Right) ||
      ImGui::IsMouseClicked(ImGuiMouseButton_Middle);
  const bool held_any =
      ImGui::IsMouseDown(ImGuiMouseButton_Right) ||
      ImGui::IsMouseDown(ImGuiMouseButton_Middle);
  if (hovered && (clicked_any || held_any)) {
    state.viewport_focused = true;
  }
  if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse &&
      (state.play_state != PlayState::Play || io.KeyCtrl)) {
    state.pick_requested = true;
    state.pick_mouse_pos[0] = io.MousePos.x;
    state.pick_mouse_pos[1] = io.MousePos.y;
  }
  if (!hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse) {
    state.viewport_focused = false;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
    state.viewport_focused = false;
    state.auto_select = false;
    state.selected_entity = rkg::ecs::kInvalidEntity;
    state.selected_name.clear();
  }
  const char* play_label = (state.play_state == PlayState::Play)
                               ? "Play"
                               : (state.play_state == PlayState::Pause ? "Pause" : "Edit");
  ImGui::SetCursorScreenPos({state.viewport_pos[0] + 8.0f, state.viewport_pos[1] + 8.0f});
  ImGui::BeginChild("ViewportHUD", ImVec2(320, 0),
                    false, ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoInputs |
                               ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  ImGui::Text("Viewport Focus: %s", state.viewport_focused ? "ON" : "OFF");
  ImGui::Text("Mode: %s", play_label);
  const auto player = state.runtime->player_entity();
  const auto& registry = state.runtime->registry();
  if (player != rkg::ecs::kInvalidEntity) {
    if (const auto* transform = registry.get_transform(player)) {
      ImGui::Text("Player Pos: %.2f %.2f %.2f",
                  transform->position[0], transform->position[1], transform->position[2]);
    }
    if (const auto* velocity = registry.get_velocity(player)) {
      ImGui::Text("Player Vel: %.2f %.2f %.2f",
                  velocity->linear[0], velocity->linear[1], velocity->linear[2]);
    }
    if (const auto* controller = registry.get_character_controller(player)) {
      ImGui::Text("Grounded: %s", controller->grounded ? "yes" : "no");
    } else {
      ImGui::TextUnformatted("Controller: (missing)");
    }
  }
  ImGui::EndChild();
  ImGui::End();
}

void draw_scene_panel(EditorState& state) {
  ImGui::Begin("Scene");
  const auto& project = state.runtime->project();
  ImGui::Text("Level: %s", project.initial_level.empty() ? "(none)" : project.initial_level.c_str());
  ImGui::Text("Entities: %zu", state.runtime->registry().entity_count());
  ImGui::Separator();

  std::unordered_map<rkg::ecs::Entity, std::string> name_lookup;
  for (const auto& [name, entity] : state.runtime->entities_by_name()) {
    name_lookup[entity] = name;
  }

  std::vector<std::pair<std::string, rkg::ecs::Entity>> entries;
  const auto entities = state.runtime->registry().entities();
  entries.reserve(entities.size());
  for (const auto entity : entities) {
    auto it = name_lookup.find(entity);
    if (it != name_lookup.end()) {
      entries.emplace_back(it->second, entity);
    } else {
      entries.emplace_back("Entity " + std::to_string(entity), entity);
    }
  }
  std::sort(entries.begin(), entries.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });

  bool selected_valid = false;
  for (const auto& entry : entries) {
    if (entry.second == state.selected_entity) {
      selected_valid = true;
      break;
    }
  }
  if (!selected_valid) {
    set_selected_entity(state, rkg::ecs::kInvalidEntity);
  }

  if (state.auto_select && state.selected_entity == rkg::ecs::kInvalidEntity && !entries.empty()) {
    set_selected_entity(state, entries.front().second);
    state.auto_select = false;
  }

  if (entries.empty()) {
    ImGui::TextUnformatted("No entities.");
  } else {
    for (const auto& entry : entries) {
      const bool selected = (entry.second == state.selected_entity);
      if (ImGui::Selectable(entry.first.c_str(), selected)) {
        set_selected_entity(state, entry.second);
        state.auto_select = false;
      }
    }
  }
  ImGui::End();
}

void draw_inspector_panel(EditorState& state) {
  ImGui::Begin("Inspector");
  const auto entity = state.selected_entity;
  if (entity == rkg::ecs::kInvalidEntity) {
    ImGui::TextUnformatted("Select an entity.");
    ImGui::End();
    return;
  }
  auto& registry = registry_mutable(state);
  ImGui::Text("Entity: %u", entity);
  if (!state.selected_name.empty()) {
    ImGui::Text("Name: %s", state.selected_name.c_str());
  }
  const std::string override_key = state.runtime->override_key_for_entity(entity);
  if (!override_key.empty()) {
    ImGui::Text("Override ID: %s", override_key.c_str());
  }
  if (const auto* transform = registry.get_transform(entity)) {
    ImGui::Separator();
    ImGui::Text("Transform");
    float pos[3] = {transform->position[0], transform->position[1], transform->position[2]};
    const bool pos_changed = ImGui::DragFloat3("Position", pos, 0.05f);
    const ImGuiID pos_id = ImGui::GetItemID();
    if (ImGui::IsItemActivated()) {
      begin_undo_edit(state.undo, pos_id, entity, registry);
    }
    if (pos_changed) {
      if (auto* writable = registry.get_transform(entity)) {
        writable->position[0] = pos[0];
        writable->position[1] = pos[1];
        writable->position[2] = pos[2];
        mark_overrides_dirty(state);
      }
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) {
      end_undo_edit(state.undo, pos_id, entity, registry);
    }
    ImGui::Text("Rotation: %.2f %.2f %.2f", transform->rotation[0], transform->rotation[1], transform->rotation[2]);
    ImGui::Text("Scale: %.2f %.2f %.2f", transform->scale[0], transform->scale[1], transform->scale[2]);
  } else {
    ImGui::TextUnformatted("Transform: (none)");
  }

  auto* renderable = registry.get_renderable(entity);
  ImGui::Separator();
  ImGui::Text("Renderable");
  if (!renderable) {
    if (ImGui::Button("Add Renderable")) {
      const EntitySnapshot before = snapshot_entity(registry, entity);
      rkg::ecs::Renderable fresh{};
      registry.set_renderable(entity, fresh);
      const EntitySnapshot after = snapshot_entity(registry, entity);
      push_undo_entry(state.undo, entity, before, after);
      mark_overrides_dirty(state);
    }
  } else {
    const char* mesh_items[] = {"cube", "quad"};
    int mesh_index = 0;
    if (renderable->mesh == rkg::ecs::MeshId::Quad) {
      mesh_index = 1;
    }
    if (ImGui::Combo("Mesh", &mesh_index, mesh_items, 2)) {
      const EntitySnapshot before = snapshot_entity(registry, entity);
      renderable->mesh = (mesh_index == 1) ? rkg::ecs::MeshId::Quad : rkg::ecs::MeshId::Cube;
      const EntitySnapshot after = snapshot_entity(registry, entity);
      push_undo_entry(state.undo, entity, before, after);
      mark_overrides_dirty(state);
    }
    float color[4] = {renderable->color[0], renderable->color[1], renderable->color[2], renderable->color[3]};
    const bool color_changed = ImGui::ColorEdit4("Color", color);
    const ImGuiID color_id = ImGui::GetItemID();
    if (ImGui::IsItemActivated()) {
      begin_undo_edit(state.undo, color_id, entity, registry);
    }
    if (color_changed) {
      renderable->color[0] = color[0];
      renderable->color[1] = color[1];
      renderable->color[2] = color[2];
      renderable->color[3] = color[3];
      mark_overrides_dirty(state);
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) {
      end_undo_edit(state.undo, color_id, entity, registry);
    }
  }

  if (auto* skeleton = registry.get_skeleton(entity)) {
    ImGui::Separator();
    ImGui::Text("Skeleton");
    ImGui::Text("Bones: %zu", skeleton->bones.size());
    if (auto* transform = registry.get_transform(entity)) {
      rkg::ecs::compute_skeleton_world_pose(*transform, *skeleton);
      if (state.selected_bone >= 0 &&
          static_cast<size_t>(state.selected_bone) < skeleton->bones.size()) {
        const auto& bone = skeleton->bones[state.selected_bone];
        const auto& world = skeleton->world_pose[state.selected_bone];
        ImGui::Text("Selected: %d (%s)", state.selected_bone,
                    bone.name.empty() ? "(unnamed)" : bone.name.c_str());
        ImGui::Text("World Pos: %.2f %.2f %.2f",
                    world.position[0], world.position[1], world.position[2]);
      }

      std::vector<std::vector<int>> children;
      children.resize(skeleton->bones.size());
      for (size_t i = 0; i < skeleton->bones.size(); ++i) {
        const int parent = skeleton->bones[i].parent_index;
        if (parent >= 0 && static_cast<size_t>(parent) < skeleton->bones.size()) {
          children[parent].push_back(static_cast<int>(i));
        }
      }

      std::function<void(int)> draw_bone = [&](int idx) {
        const auto& bone = skeleton->bones[idx];
        const std::string label =
            bone.name.empty() ? std::string("bone_") + std::to_string(idx) : bone.name;
        const bool is_selected = state.selected_bone == idx;
        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                                   ImGuiTreeNodeFlags_SpanFullWidth;
        if (children[idx].empty()) {
          flags |= ImGuiTreeNodeFlags_Leaf;
        }
        if (is_selected) {
          flags |= ImGuiTreeNodeFlags_Selected;
        }
        const bool open = ImGui::TreeNodeEx(reinterpret_cast<void*>(static_cast<intptr_t>(idx)),
                                            flags, "%s", label.c_str());
        if (ImGui::IsItemClicked()) {
          state.selected_bone = idx;
        }
        if (open) {
          for (const int child : children[idx]) {
            draw_bone(child);
          }
          ImGui::TreePop();
        }
      };

      for (size_t i = 0; i < skeleton->bones.size(); ++i) {
        if (skeleton->bones[i].parent_index < 0) {
          draw_bone(static_cast<int>(i));
        }
      }
    } else {
      ImGui::TextUnformatted("Transform required for world pose.");
    }
  }

  ImGui::Separator();
  ImGui::BeginDisabled(override_key.empty());
  if (ImGui::Button("Revert Selected")) {
    revert_selected(state);
  }
  ImGui::EndDisabled();
  if (override_key.empty()) {
    ImGui::TextUnformatted("Revert Selected: missing override id");
  }
  ImGui::End();
}

void draw_content_panel(EditorState& state) {
  ImGui::Begin("Content");
  ImGui::TextWrapped("Project: %s", state.runtime->project_root().generic_string().c_str());
  ImGui::TextWrapped("Cooked root: %s", state.runtime->cooked_root().generic_string().c_str());
  ImGui::TextWrapped("Pack: %s", state.runtime->pack_path().generic_string().c_str());
  ImGui::Text("Cook status: %s", state.runtime->last_cook_success().empty()
                                    ? "(none)"
                                    : state.runtime->last_cook_success().c_str());
  if (!state.runtime->last_cook_error().empty()) {
    ImGui::TextWrapped("Cook error: %s", state.runtime->last_cook_error().c_str());
  }
  ImGui::Text("Reload: %s", state.runtime->last_reload_time().c_str());
  if (!state.runtime->last_reload_error().empty()) {
    ImGui::TextWrapped("Reload error: %s", state.runtime->last_reload_error().c_str());
  }
  ImGui::Separator();

#if RKG_ENABLE_DATA_YAML
  ImGui::TextUnformatted("Editor Overrides");
  if (!state.overrides.path.empty()) {
    ImGui::TextWrapped("Overrides file: %s", state.overrides.path.generic_string().c_str());
  }
  size_t total_overrides = 0;
  const size_t dirty_overrides = count_dirty_overrides(state, &total_overrides);
  ImGui::Text("Overrides: %zu (dirty %zu)", total_overrides, dirty_overrides);
  if (state.overrides.dirty) {
    ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "Status: dirty");
  } else {
    ImGui::TextUnformatted("Status: clean");
  }
  if (ImGui::Button("Save Overrides")) {
    save_overrides(state);
  }
  ImGui::SameLine();
  if (ImGui::Button("Revert All")) {
    revert_all(state);
  }
  ImGui::Separator();
  ImGui::TextUnformatted("Commit Overrides (stage -> review -> apply)");
  const std::string level_arg = current_level_arg(*state.runtime);
  ImGui::TextWrapped("Target Level File: %s", level_arg.empty() ? "(unknown)" : level_arg.c_str());
  const std::string selected_key = state.runtime->override_key_for_entity(state.selected_entity);
  const std::string selected_name = state.selected_name;
  ImGui::TextUnformatted("Stage Mode:");
  const int stage_mode = state.content.commit_stage_selected_only ? 1 : 0;
  if (ImGui::RadioButton("Stage All Overrides", stage_mode == 0)) {
    state.content.commit_stage_selected_only = false;
  }
  ImGui::BeginDisabled(selected_key.empty() && selected_name.empty());
  if (ImGui::RadioButton("Stage Selected Only", stage_mode == 1)) {
    state.content.commit_stage_selected_only = true;
  }
  ImGui::EndDisabled();
  if (state.content.commit_stage_selected_only) {
    ImGui::TextWrapped("Selected override id: %s", selected_key.empty() ? "(none)" : selected_key.c_str());
    if (selected_key.empty() && !selected_name.empty()) {
      ImGui::TextWrapped("Selected name: %s", selected_name.c_str());
    }
    if (selected_key.empty() && !selected_name.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f),
                         "Warning: staging by name is less reliable; add id to level YAML.");
    }
  }
  const bool stage_blocked = state.overrides.dirty || state.content.commit_process.running();
  ImGui::BeginDisabled(stage_blocked);
  if (ImGui::Button("Stage Overrides (Dry-run)")) {
    reset_diff_preview(state.diff_preview);
    if (state.content.commit_stage_selected_only && selected_key.empty() && selected_name.empty()) {
      state.content.commit_error = "selected entity has no id or name";
    } else {
      const std::string entity_id = state.content.commit_stage_selected_only ? selected_key : std::string{};
      const std::string entity_name =
          (state.content.commit_stage_selected_only && selected_key.empty()) ? selected_name : std::string{};
      start_commit_stage(state.content, *state.runtime, entity_id, entity_name);
    }
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  const bool staged_ready = (state.content.commit_stage == "stage" &&
                             state.content.commit_status == "ok" &&
                             !state.content.commit_run_dir.empty());
  ImGui::BeginDisabled(state.content.commit_process.running() || !staged_ready);
  if (ImGui::Button("Apply Staged")) {
    start_commit_apply_staged(state.content, *state.runtime, state.content.commit_run_dir, false);
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(state.content.commit_process.running() || !staged_ready);
  if (ImGui::Button("Apply Staged (Force)")) {
    state.content.commit_force_modal = true;
  }
  ImGui::EndDisabled();
  if (state.content.commit_force_modal) {
    ImGui::OpenPopup("Force Apply Staged");
  }
  if (ImGui::BeginPopupModal("Force Apply Staged", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted("This may overwrite changes. Proceed?");
    ImGui::Separator();
    if (ImGui::Button("Apply Force")) {
      start_commit_apply_staged(state.content, *state.runtime, state.content.commit_run_dir, true);
      state.content.commit_force_modal = false;
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      state.content.commit_force_modal = false;
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
  if (state.overrides.dirty) {
    ImGui::TextUnformatted("Save overrides before staging.");
  } else if (!staged_ready) {
    ImGui::TextUnformatted("Stage overrides to review diffs before applying.");
  } else {
    ImGui::TextUnformatted("Review staged diffs in the Diff Preview panel before applying.");
  }
  if (!state.content.commit_error.empty()) {
    ImGui::TextWrapped("Commit error: %s", state.content.commit_error.c_str());
  }
  if (!state.content.commit_status.empty()) {
    ImGui::Text("Commit status: %s", state.content.commit_status.c_str());
    ImGui::Text("Dry-run: %s", state.content.commit_dry_run ? "yes" : "no");
    if (!state.content.commit_stage.empty()) {
      ImGui::Text("Stage: %s", state.content.commit_stage.c_str());
    }
    if (state.content.commit_conflicts > 0) {
      ImGui::Text("Conflicts: %d", state.content.commit_conflicts);
    }
    if (!state.content.commit_error_code.empty() || !state.content.commit_error_message.empty()) {
      ImGui::TextWrapped("Error: %s %s", state.content.commit_error_code.c_str(),
                         state.content.commit_error_message.c_str());
    }
    if (!state.content.commit_selector_warning.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s",
                         state.content.commit_selector_warning.c_str());
    }
    if (state.content.commit_conflict_detected) {
      ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "Conflict detected: yes");
    }
    if (state.content.commit_forced_apply) {
      ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "Forced apply: yes");
    }
    if (!state.content.commit_entity_id.empty()) {
      ImGui::Text("Entity: %s", state.content.commit_entity_id.c_str());
    }
    if (!state.content.commit_run_dir.empty()) {
      ImGui::TextWrapped("Run dir: %s", state.content.commit_run_dir.c_str());
    }
    if (!state.content.commit_staging_dir.empty()) {
      ImGui::TextWrapped("Staging dir: %s", state.content.commit_staging_dir.c_str());
    }
  }
  if (state.diff_preview.loaded) {
    ImGui::Separator();
    ImGui::Text("Last Staged Run: %s", state.diff_preview.run_id.empty() ? "(unknown)" : state.diff_preview.run_id.c_str());
    ImGui::SameLine();
    if (ImGui::SmallButton("Clear Selection")) {
      reset_diff_preview(state.diff_preview);
    }
  }
  if (!state.overrides.last_error.empty()) {
    ImGui::TextWrapped("Override error: %s", state.overrides.last_error.c_str());
  }
  ImGui::Separator();
#else
  ImGui::TextUnformatted("Editor Overrides: unavailable (YAML disabled)");
  ImGui::Separator();
#endif

  ImGui::BeginDisabled(state.content.cook_process.running());
  if (ImGui::Button("Cook Now")) {
    start_cook(state.content, *state.runtime);
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  const bool watching = state.content.watch_process.running();
  if (!watching) {
    if (ImGui::Button("Watch")) {
      start_watch(state.content, *state.runtime);
    }
  } else {
    if (ImGui::Button("Stop Watch")) {
      state.content.watch_process.request_stop();
    }
  }

  if (!state.content.last_error.empty()) {
    ImGui::TextWrapped("Error: %s", state.content.last_error.c_str());
  }

  if (state.content.cook_process.running() || state.content.watch_process.running()) {
    ImGui::Separator();
    ImGui::TextUnformatted("Content logs:");
    ImGui::BeginChild("content_logs", ImVec2(0, 120), true);
    const auto lines = state.content.watch_process.running()
                           ? command_lines(state.content.watch_process, 100)
                           : command_lines(state.content.cook_process, 100);
    for (const auto& line : lines) {
      ImGui::TextUnformatted(line.c_str());
    }
    ImGui::EndChild();
  }

  if (state.content.commit_process.running()) {
    ImGui::Separator();
    ImGui::TextUnformatted("Commit logs:");
    ImGui::BeginChild("commit_logs", ImVec2(0, 120), true);
    const auto lines = command_lines(state.content.commit_process, 100);
    for (const auto& line : lines) {
      ImGui::TextUnformatted(line.c_str());
    }
    ImGui::EndChild();
  }

  ImGui::End();
}

void draw_diff_preview_panel(EditorState& state) {
  ImGui::Begin("Diff Preview");
#if RKG_ENABLE_DATA_JSON
  if (!state.diff_preview.error.empty()) {
    ImGui::TextWrapped("Error: %s", state.diff_preview.error.c_str());
  }

  if (!state.diff_preview.loaded) {
    ImGui::TextUnformatted("No staged diff loaded.");
    ImGui::TextUnformatted("Stage overrides to generate a diff for review.");
    ImGui::End();
    return;
  }

  ImGui::Text("Run: %s", state.diff_preview.run_id.empty() ? "(unknown)" : state.diff_preview.run_id.c_str());
  if (!state.diff_preview.level_path.empty()) {
    ImGui::TextWrapped("Target Level: %s", state.diff_preview.level_path.c_str());
  }
  if (!state.diff_preview.selector_type.empty() && state.diff_preview.selector_type != "all") {
    ImGui::TextWrapped("Selector: %s (%s)", state.diff_preview.selector_type.c_str(),
                       state.diff_preview.selector_value.c_str());
  } else if (!state.diff_preview.entity_id.empty()) {
    ImGui::TextWrapped("Target Entity: %s", state.diff_preview.entity_id.c_str());
  }
  ImGui::Text("Files: %zu", state.diff_preview.diffs.size());
  if (state.content.commit_error_code == "conflict") {
    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f),
                       "CONFLICT: apply blocked until restaged or forced");
  }
  if (state.content.commit_conflicts > 0) {
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Conflicts: %d",
                       state.content.commit_conflicts);
  }
  if (!state.content.commit_error_code.empty() || !state.content.commit_error_message.empty()) {
    ImGui::TextWrapped("Status: %s %s", state.content.commit_error_code.c_str(),
                       state.content.commit_error_message.c_str());
  }
  if (!state.diff_preview.selector_warning.empty()) {
    ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", state.diff_preview.selector_warning.c_str());
  }
  if (!state.content.commit_run_dir.empty()) {
    ImGui::TextWrapped("Run dir: %s", state.content.commit_run_dir.c_str());
    draw_copy_button("Copy##commit_run_dir", state.content.commit_run_dir);
  }

  if (state.diff_preview.diffs.empty()) {
    ImGui::Separator();
    ImGui::TextUnformatted("No file changes staged.");
    ImGui::End();
    return;
  }

  if (!state.content.commit_conflict_files.empty()) {
    ImGui::Separator();
    ImGui::TextUnformatted("Conflict files:");
    for (size_t i = 0; i < state.content.commit_conflict_files.size(); ++i) {
      const auto& path = state.content.commit_conflict_files[i];
      ImGui::PushID(static_cast<int>(i));
      ImGui::TextWrapped("%s", path.c_str());
      draw_copy_button("Copy", path);
      ImGui::SameLine();
      if (ImGui::SmallButton("Open")) {
        const fs::path abs_path = state.runtime->paths().root / path;
        start_open_file(state.content, abs_path.generic_string(), *state.runtime);
      }
      ImGui::PopID();
    }
    if (!state.content.open_file_error.empty()) {
      ImGui::TextWrapped("Open file error: %s", state.content.open_file_error.c_str());
    }
  }

  ImGui::Separator();
  const bool truncated = draw_diff_entries_viewer(state.diff_preview.diffs, state.diff_preview.selected,
                                                  "diff_list", "diff_text");
  if (truncated) {
    ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "Diff truncated for display.");
  }
#else
  ImGui::TextUnformatted("Diff preview unavailable (JSON disabled).");
#endif
  ImGui::End();
}

void draw_chat_panel(EditorState& state) {
  auto& agent = state.agent;
  ImGui::Begin("Chat");
  state.chat_active = false;

  if (!agent.openai_available) {
    ImGui::TextWrapped("%s", agent.openai_error.c_str());
  }

  ImGui::InputTextMultiline("Goal", agent.goal_buffer, sizeof(agent.goal_buffer), ImVec2(0, 80));
  agent.goal = agent.goal_buffer;
  if (ImGui::IsItemActive()) {
    state.chat_active = true;
  }
  const bool running = agent.stage != AgentPanelState::Stage::Idle;
  ImGui::BeginDisabled(running || !agent.openai_available);
  if (ImGui::Button("Plan")) {
    start_agent_plan(state);
  }
  ImGui::EndDisabled();

  if (agent.plan_summary.valid) {
    ImGui::Separator();
    ImGui::Text("Plan tasks: %zu", agent.plan_summary.task_count);
    for (const auto& task : agent.plan_summary.top_tasks) {
      ImGui::BulletText("%s", task.c_str());
    }
  } else if (!agent.plan_summary.error.empty()) {
    ImGui::TextWrapped("Plan summary error: %s", agent.plan_summary.error.c_str());
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Apply (explicit confirmation)");
  ImGui::InputText("Plan path", agent.plan_path_buffer, sizeof(agent.plan_path_buffer));
  agent.plan_path = agent.plan_path_buffer;
  if (ImGui::IsItemActive()) {
    state.chat_active = true;
  }
  ImGui::BeginDisabled(running || agent.plan_path.empty());
  if (ImGui::Button("Apply")) {
    start_agent_apply(state, agent.plan_path);
  }
  ImGui::EndDisabled();

  ImGui::Separator();
  ImGui::TextUnformatted("Offline Apply (use a plan file)");
  ImGui::InputText("Offline plan", agent.offline_plan_buffer, sizeof(agent.offline_plan_buffer));
  agent.offline_plan_path = agent.offline_plan_buffer;
  if (ImGui::IsItemActive()) {
    state.chat_active = true;
  }
  ImGui::BeginDisabled(running || agent.offline_plan_path.empty());
  if (ImGui::Button("Apply Offline")) {
    start_agent_apply(state, agent.offline_plan_path);
  }
  ImGui::EndDisabled();

  if (!agent.last_error.empty()) {
    ImGui::TextWrapped("Error: %s", agent.last_error.c_str());
  }

  if (running) {
    ImGui::Separator();
    const char spinner_chars[] = "|/-\\";
    const int spin = static_cast<int>(ImGui::GetTime() * 4.0) % 4;
    ImGui::Text("Running %c  %s", spinner_chars[spin], agent.last_command_label.c_str());
    const float progress = agent.stage == AgentPanelState::Stage::RunningStatus ? 0.85f : 0.35f;
    ImGui::ProgressBar(progress, ImVec2(-1, 0), nullptr);
    ImGui::BeginChild("agent_logs", ImVec2(0, 150), true);
    const auto lines = agent.stage == AgentPanelState::Stage::RunningStatus
                           ? command_lines(agent.status_process, 100)
                           : command_lines(agent.agent_process, 100);
    for (const auto& line : lines) {
      ImGui::TextUnformatted(line.c_str());
    }
    ImGui::EndChild();
  }

  ImGui::End();
}

[[maybe_unused]] void draw_agent_runs_panel(EditorState& state) {
  ImGui::Begin("Agent Runs");
  const auto& status = state.agent.status;
  if (!status.valid) {
    ImGui::TextUnformatted("No agent status yet.");
    ImGui::End();
    return;
  }
  if (!status.provider.empty()) {
    ImGui::Text("Provider: %s", status.provider.c_str());
  }
  if (!status.provider_note.empty()) {
    ImGui::TextWrapped("Note: %s", status.provider_note.c_str());
  }
  if (!status.model.empty()) {
    ImGui::Text("Model: %s", status.model.c_str());
  }

  ImGui::Separator();
  const auto& run = status.last_run;
  if (!run.valid) {
    ImGui::TextUnformatted("No runs found.");
    ImGui::End();
    return;
  }
  ImGui::Text("Run: %s", run.run_id.c_str());
  if (!run.created_at.empty()) {
    ImGui::Text("Created: %s", run.created_at.c_str());
  }
  if (!run.goal.empty()) {
    ImGui::TextWrapped("Goal: %s", run.goal.c_str());
  }
  if (!run.selector_type.empty() && run.selector_type != "all") {
    ImGui::TextWrapped("Selector: %s (%s)", run.selector_type.c_str(), run.selector_value.c_str());
  }
  if (!run.selector_warning.empty()) {
    ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", run.selector_warning.c_str());
  }
  if (!run.entity_id.empty()) {
    ImGui::TextWrapped("Entity id: %s", run.entity_id.c_str());
  }
  if (!run.entity_name.empty()) {
    ImGui::TextWrapped("Entity name: %s", run.entity_name.c_str());
  }
  if (!run.status.empty()) {
    ImGui::Text("Status: %s", run.status.c_str());
  }
  ImGui::Text("Conflicts: %d", run.conflicts);
  ImGui::Text("Mode: %s", run.dry_run ? "dry-run" : "applied");
  if (run.drift_detected) {
    ImGui::Text("Drift: mismatch (+%d -%d ~%d)", run.drift_added, run.drift_removed, run.drift_modified);
  } else {
    ImGui::Text("Drift: match");
  }
  if (!run.drift_message.empty()) {
    ImGui::TextWrapped("Drift note: %s", run.drift_message.c_str());
  }

  if (!run.error_message.empty() || !run.error_code.empty()) {
    ImGui::Separator();
    ImGui::TextWrapped("Last error: %s %s", run.error_code.c_str(), run.error_message.c_str());
    if (!run.error_stage.empty()) {
      ImGui::Text("Stage: %s", run.error_stage.c_str());
    }
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Run timeline");
  const fs::path run_dir_path = run.run_dir.empty() ? fs::path{} : fs::path(run.run_dir);
  const bool plan_exists = !run.plan_path.empty() && fs::exists(run.plan_path);
  const bool results_exists =
      !run.run_dir.empty() && fs::exists(run_dir_path / "results.json");
  const char* stages[] = {"plan", "validate", "stage", "apply", "cook", "reload"};
  for (const char* stage : stages) {
    const StageStatus status = status_for_stage(run, run_dir_path, stage, plan_exists, results_exists);
    ImGui::Text("%s %s", stage_icon(status), stage);
  }

  ImGui::Separator();
  if (!run.run_dir.empty()) {
    ImGui::TextWrapped("Run dir: %s", run.run_dir.c_str());
    draw_copy_button("Copy##run_dir", run.run_dir);
    ImGui::SameLine();
    if (ImGui::SmallButton("Open##run_dir")) {
      start_open_folder(state.agent, run.run_dir, *state.runtime);
    }
  }
  if (!run.plan_path.empty()) {
    ImGui::TextWrapped("Plan: %s", run.plan_path.c_str());
    draw_copy_button("Copy##plan", run.plan_path);
  }
  if (!run.run_dir.empty()) {
    const std::string results_path = (fs::path(run.run_dir) / "results.json").generic_string();
    const std::string ctx_path = (fs::path(run.run_dir) / "context_pack.json").generic_string();
    const std::string openai_path = (fs::path(run.run_dir) / "openai_response.json").generic_string();
    ImGui::TextWrapped("Results: %s", results_path.c_str());
    draw_copy_button("Copy##results", results_path);
    if (fs::exists(openai_path)) {
      ImGui::TextWrapped("OpenAI response: %s", openai_path.c_str());
      draw_copy_button("Copy##openai", openai_path);
    }
    ImGui::TextWrapped("Context: %s", ctx_path.c_str());
    draw_copy_button("Copy##context", ctx_path);
  }
  if (!state.agent.open_error.empty()) {
    ImGui::TextWrapped("Open folder error: %s", state.agent.open_error.c_str());
  }

  ImGui::End();
}

void draw_runs_browser_panel(EditorState& state) {
  ImGui::Begin("Runs");
#if RKG_ENABLE_DATA_JSON
  auto& runs = state.runs;
  if (!runs.runs_error.empty()) {
    ImGui::TextWrapped("Run scan error: %s", runs.runs_error.c_str());
  }

  ImGui::InputText("Search", runs.search_buffer, sizeof(runs.search_buffer));
  ImGui::SameLine();
  if (ImGui::Button("Cleanup Runs...")) {
    runs.cleanup_modal = true;
  }
  ImGui::Checkbox("AI", &runs.filter_ai);
  ImGui::SameLine();
  ImGui::Checkbox("Commit", &runs.filter_commit);
  ImGui::SameLine();
  ImGui::Checkbox("Snapshot", &runs.filter_snapshot);
  ImGui::SameLine();
  ImGui::Checkbox("Other", &runs.filter_other);
  ImGui::Checkbox("Success", &runs.status_success);
  ImGui::SameLine();
  ImGui::Checkbox("Failed", &runs.status_failed);
  ImGui::SameLine();
  ImGui::Checkbox("Conflict", &runs.status_conflict);
  ImGui::SameLine();
  ImGui::Checkbox("Unknown", &runs.status_unknown);
  if (!runs.cleanup_error.empty()) {
    ImGui::TextWrapped("%s", runs.cleanup_error.c_str());
  }

  if (runs.cleanup_modal) {
    ImGui::OpenPopup("Cleanup Runs");
  }
  if (ImGui::BeginPopupModal("Cleanup Runs", &runs.cleanup_modal, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted("Delete old runs with safeguards:");
    ImGui::InputInt("Keep last N", &runs.cleanup_keep_last);
    ImGui::InputInt("Delete older than days", &runs.cleanup_days);
    runs.cleanup_keep_last = std::max(0, runs.cleanup_keep_last);
    runs.cleanup_days = std::max(0, runs.cleanup_days);
    const rkg::RunCleanupOptions opts{static_cast<size_t>(runs.cleanup_keep_last), runs.cleanup_days};
    const fs::path selected_dir = runs.selected_run_dir.empty() ? fs::path{} : fs::path(runs.selected_run_dir);
    const auto preview = rkg::collect_run_cleanup_candidates(state.runtime->paths().root, opts, selected_dir);
    runs.cleanup_candidates = preview.candidates;
    runs.cleanup_error = preview.error;
    ImGui::Text("Will delete %zu runs.", runs.cleanup_candidates.size());
    ImGui::TextUnformatted("Safety: snapshot runs and the selected run are preserved.");
    if (!runs.cleanup_error.empty()) {
      ImGui::TextWrapped("Cleanup error: %s", runs.cleanup_error.c_str());
    }
    if (ImGui::Button("Delete")) {
      size_t removed = 0;
      for (const auto& path : runs.cleanup_candidates) {
        std::error_code ec;
        fs::remove_all(path, ec);
        if (!ec) ++removed;
      }
      refresh_runs_browser(state, true);
      runs.cleanup_modal = false;
      runs.cleanup_error = "Removed " + std::to_string(removed) + " run(s).";
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      runs.cleanup_modal = false;
    }
    ImGui::EndPopup();
  }

  ImGui::BeginChild("runs_list", ImVec2(260, 0), true);
  if (runs.runs.empty()) {
    ImGui::TextUnformatted("No runs found.");
  } else {
    bool any = false;
    for (size_t i = 0; i < runs.runs.size(); ++i) {
      const auto& run = runs.runs[i];
      if (!run_matches_filters(run, runs)) {
        continue;
      }
      any = true;
      const std::string goal = run.goal.empty() ? "(no goal)" : run.goal;
      std::string label = std::string(run_status_icon(run)) + " " + run.run_type;
      if (!run.timestamp.empty()) {
        label += " ";
        label += run.timestamp;
      }
      label += " - ";
      label += truncate_text(goal, 60);
      if (ImGui::Selectable(label.c_str(), runs.selected == static_cast<int>(i))) {
        runs.selected = static_cast<int>(i);
        runs.selected_run_dir = run.run_dir;
        runs.diffs_loaded = false;
        runs.diff_error.clear();
        runs.diff_title.clear();
        runs.diff_is_snapshot = false;
        runs.diffs.clear();
      }
    }
    if (!any) {
      ImGui::TextUnformatted("No runs match filters.");
    }
  }
  ImGui::EndChild();
  ImGui::SameLine();

  ImGui::BeginChild("runs_detail", ImVec2(0, 0), true);
  if (runs.runs.empty() || runs.selected < 0 || runs.selected >= static_cast<int>(runs.runs.size())) {
    ImGui::TextUnformatted("Select a run to view details.");
    ImGui::EndChild();
    ImGui::End();
    return;
  }

  const auto& run = runs.runs[runs.selected];
  if (run.run_dir != runs.selected_run_dir) {
    runs.selected_run_dir = run.run_dir;
    runs.diffs_loaded = false;
    runs.diff_error.clear();
    runs.diff_title.clear();
    runs.diff_is_snapshot = false;
    runs.diffs.clear();
  }
  if (!runs.diffs_loaded && !run.staged_patches.empty()) {
    load_run_diffs(run, runs, runs.diff_error);
  }

  ImGui::Text("Run: %s", run.run_id.c_str());
  ImGui::Text("Type: %s", run.run_type.c_str());
  if (!run.timestamp.empty()) {
    ImGui::Text("Time: %s", run.timestamp.c_str());
  }
  if (!run.project_path.empty()) {
    ImGui::TextWrapped("Project: %s", run.project_path.c_str());
  }
  if (!run.level_path.empty()) {
    ImGui::TextWrapped("Level: %s", run.level_path.c_str());
  }
  if (!run.overrides_path.empty()) {
    ImGui::TextWrapped("Overrides: %s", run.overrides_path.c_str());
  }
  if (!run.snapshot_run_id.empty()) {
    ImGui::TextWrapped("Snapshot run: %s", run.snapshot_run_id.c_str());
  }
  if (!run.snapshot_path.empty()) {
    ImGui::TextWrapped("Snapshot file: %s", run.snapshot_path.c_str());
  }
  if (!run.goal.empty()) {
    ImGui::TextWrapped("Goal: %s", run.goal.c_str());
  }
  ImGui::Text("Status: %s", run.status.empty() ? (run.success ? "ok" : "unknown") : run.status.c_str());
  if (!run.error_stage.empty()) {
    ImGui::Text("Stage: %s", run.error_stage.c_str());
  }
  if (!run.error_code.empty() || !run.error_message.empty()) {
    ImGui::TextWrapped("Error: %s %s", run.error_code.c_str(), run.error_message.c_str());
  }

  ImGui::Separator();
  ImGui::TextWrapped("Run dir: %s", run.run_dir.c_str());
  draw_copy_button("Copy##run_dir_runs", run.run_dir);
  ImGui::SameLine();
  if (ImGui::SmallButton("Open##run_dir_runs")) {
    start_open_folder(runs, run.run_dir, *state.runtime);
  }
  if (!runs.open_error.empty()) {
    ImGui::TextWrapped("Open folder error: %s", runs.open_error.c_str());
  }

  if (!run.staged_patches.empty()) {
    ImGui::TextWrapped("Staged patches: %s", run.staged_patches.c_str());
    draw_copy_button("Copy##staged_dir_runs", run.staged_patches);
  }

  if (run.error_code == "conflict") {
    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f),
                       "CONFLICT: apply blocked until restaged or forced");
  }

  if (!run.conflict_files.empty()) {
    ImGui::Separator();
    ImGui::TextUnformatted("Conflicts:");
    for (size_t i = 0; i < run.conflict_files.size(); ++i) {
      const auto& path = run.conflict_files[i];
      ImGui::PushID(static_cast<int>(i));
      ImGui::TextWrapped("%s", path.c_str());
      draw_copy_button("Copy", path);
      ImGui::SameLine();
      if (ImGui::SmallButton("Open")) {
        fs::path abs_path = fs::path(path);
        if (!abs_path.is_absolute()) {
          abs_path = state.runtime->paths().root / path;
        }
        start_open_file(runs, abs_path.generic_string(), *state.runtime);
      }
      ImGui::PopID();
    }
    if (!runs.open_file_error.empty()) {
      ImGui::TextWrapped("Open file error: %s", runs.open_file_error.c_str());
    }
  }

  if (run.snapshots_taken || !run.snapshot_manifest_path.empty()) {
    ImGui::Separator();
    ImGui::TextUnformatted("Snapshots:");
    if (!run.snapshot_manifest_path.empty()) {
      ImGui::TextWrapped("Manifest: %s", run.snapshot_manifest_path.c_str());
      draw_copy_button("Copy##snapshot_manifest", run.snapshot_manifest_path);
      ImGui::SameLine();
      if (ImGui::SmallButton("Open##snapshot_manifest")) {
        start_open_file(runs, run.snapshot_manifest_path, *state.runtime);
      }
    }
    if (!run.snapshots.empty()) {
      ImGui::BeginChild("snapshot_list", ImVec2(0, 160), true);
      for (size_t i = 0; i < run.snapshots.size(); ++i) {
        const auto& entry = run.snapshots[i];
        ImGui::PushID(static_cast<int>(i));
        ImGui::TextWrapped("%s (%llu)", entry.path.c_str(),
                           static_cast<unsigned long long>(entry.size));
        if (ImGui::SmallButton("Compare to current")) {
          DiffPreviewEntry diff_entry;
          bool current_missing = false;
          std::string diff_err;
          if (build_snapshot_compare_diff(entry, state.runtime->paths().root, diff_entry,
                                          current_missing, diff_err)) {
            runs.diffs.clear();
            runs.diffs.push_back(diff_entry);
            runs.diff_selected = 0;
            runs.diffs_loaded = true;
            runs.diff_is_snapshot = true;
            runs.diff_title = current_missing ? "Snapshot vs current (missing)" : "Snapshot vs current";
            runs.diff_error.clear();
          } else {
            runs.diff_error = diff_err;
          }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Stage Restore")) {
          if (stage_snapshot_restore_run(runs, entry, run, *state.runtime)) {
            refresh_runs_browser(state, true);
            for (size_t idx = 0; idx < runs.runs.size(); ++idx) {
              if (runs.runs[idx].run_dir == runs.selected_run_dir) {
                runs.selected = static_cast<int>(idx);
                runs.diffs_loaded = false;
                runs.diff_error.clear();
                break;
              }
            }
          }
        }
        if (!entry.hash.empty()) {
          ImGui::Text("hash: %s", entry.hash.c_str());
        }
        if (!entry.snapshot_path.empty()) {
          ImGui::TextWrapped("snapshot: %s", entry.snapshot_path.c_str());
        }
        ImGui::PopID();
      }
      ImGui::EndChild();
    }
    if (!runs.open_file_error.empty()) {
      ImGui::TextWrapped("Open snapshot error: %s", runs.open_file_error.c_str());
    }
    if (!runs.restore_error.empty()) {
      ImGui::TextWrapped("Snapshot restore error: %s", runs.restore_error.c_str());
    }
  }

  if (!run.target_files.empty()) {
    ImGui::Separator();
    ImGui::Text("Targets: %zu", run.target_files.size());
    ImGui::BeginChild("target_files", ImVec2(0, 100), true);
    for (const auto& path : run.target_files) {
      ImGui::TextUnformatted(path.c_str());
    }
    ImGui::EndChild();
  }

  if (run.error_code == "conflict" &&
      (run.run_type == "commit_overrides_stage" || run.run_type == "commit_overrides_apply")) {
    ImGui::Separator();
    ImGui::BeginDisabled(runs.restage_process.running());
    if (ImGui::Button("Restage")) {
      start_restage_commit(runs, run, *state.runtime);
    }
    ImGui::EndDisabled();
    if (runs.restage_process.running()) {
      ImGui::TextUnformatted("Restaging...");
      ImGui::BeginChild("restage_logs", ImVec2(0, 120), true);
      const auto lines = command_lines(runs.restage_process, 100);
      for (const auto& line : lines) {
        ImGui::TextUnformatted(line.c_str());
      }
      ImGui::EndChild();
    }
    if (!runs.restage_error.empty()) {
      ImGui::TextWrapped("Restage error: %s", runs.restage_error.c_str());
    }
    if (!runs.restage_output_error.empty()) {
      ImGui::TextWrapped("Restage output error: %s", runs.restage_output_error.c_str());
    }
  } else if (run.error_code == "conflict" &&
             (run.run_type == "agent_apply" || run.run_type == "offline_apply")) {
    ImGui::Separator();
    if (ImGui::Button("Generate new plan")) {
      const std::string prompt = "Resolve conflicts for run " + run.run_id;
      std::snprintf(state.agent.goal_buffer, sizeof(state.agent.goal_buffer), "%s", prompt.c_str());
      state.agent.goal = prompt;
    }
  }

  if (run.run_type == "snapshot_restore_stage") {
    ImGui::Separator();
    if (ImGui::Button("Apply Staged Restore")) {
      runs.restore_apply_modal = true;
      runs.restore_apply_run_dir = run.run_dir;
    }
  }

  if (runs.restore_apply_modal) {
    ImGui::OpenPopup("Apply Snapshot Restore");
  }
  if (ImGui::BeginPopupModal("Apply Snapshot Restore", &runs.restore_apply_modal,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted("Apply staged snapshot restore?");
    ImGui::TextUnformatted("This will overwrite the target file if no conflicts are found.");
    if (ImGui::Button("Apply")) {
      RunManifest apply_run = run;
      if (!runs.restore_apply_run_dir.empty()) {
        for (const auto& candidate : runs.runs) {
          if (candidate.run_dir == runs.restore_apply_run_dir) {
            apply_run = candidate;
            break;
          }
        }
      }
      if (apply_snapshot_restore_run(runs, apply_run, *state.runtime)) {
        runs.selected_run_dir = apply_run.run_dir;
        refresh_runs_browser(state, true);
      }
      runs.restore_apply_modal = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      runs.restore_apply_modal = false;
    }
    ImGui::EndPopup();
  }
  if (!runs.restore_error.empty()) {
    ImGui::TextWrapped("Restore error: %s", runs.restore_error.c_str());
  }

  ImGui::Separator();
  const std::string diff_label = runs.diff_title.empty() ? "Diff" : runs.diff_title;
  ImGui::TextUnformatted(diff_label.c_str());
  if (!runs.diffs.empty()) {
    const bool truncated = draw_diff_entries_viewer(runs.diffs, runs.diff_selected,
                                                    "runs_diff_list", "runs_diff_text");
    if (truncated) {
      ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "Diff truncated for display.");
    }
    if (!run.staged_patches.empty()) {
      if (ImGui::SmallButton("Reload staged diffs")) {
        runs.diffs_loaded = false;
        runs.diff_error.clear();
        load_run_diffs(run, runs, runs.diff_error);
      }
    }
  } else if (!runs.diff_error.empty()) {
    ImGui::TextWrapped("Diff error: %s", runs.diff_error.c_str());
  } else {
    ImGui::TextUnformatted("No staged diff available.");
  }

  ImGui::EndChild();
#else
  ImGui::TextUnformatted("Runs browser unavailable (JSON disabled).");
#endif
  ImGui::End();
}

void update_camera_and_draw_list(EditorState& state) {
  int viewport_w = 0;
  int viewport_h = 0;
  rkg::debug_ui::viewport_size(&viewport_w, &viewport_h);
  if (viewport_w <= 0 || viewport_h <= 0) {
    return;
  }

  const ImGuiIO& io = ImGui::GetIO();
  const bool can_control = state.viewport_focused && !state.ui_capturing;
  auto& registry = registry_mutable(state);

  if (registry.entity_count() == 0) {
    // Fallback starter cube when content-driven entities are unavailable.
    if (state.fallback_entity == rkg::ecs::kInvalidEntity ||
        !registry.get_transform(state.fallback_entity)) {
      const auto entity = registry.create_entity();
      rkg::ecs::Transform transform{};
      registry.set_transform(entity, transform);
      rkg::ecs::Renderable renderable{};
      registry.set_renderable(entity, renderable);
      state.fallback_entity = entity;
    }
  }

  const auto player = state.runtime->player_entity();
  if (state.play_state == PlayState::Play && can_control) {
    if (player == rkg::ecs::kInvalidEntity || !registry.get_transform(player)) {
      if (auto* transform = registry.get_transform(state.fallback_entity)) {
        const float speed = 1.5f * io.DeltaTime;
        if (state.runtime->input_action("MoveForward").held) transform->position[2] += speed;
        if (state.runtime->input_action("MoveBack").held) transform->position[2] -= speed;
        if (state.runtime->input_action("MoveLeft").held) transform->position[0] -= speed;
        if (state.runtime->input_action("MoveRight").held) transform->position[0] += speed;
      }
    }
  }

  if (state.play_state == PlayState::Play && state.viewport_focused && !state.chat_active &&
      player != rkg::ecs::kInvalidEntity) {
    if (auto* transform = registry.get_transform(player)) {
      transform->rotation[1] = state.camera_yaw;
    }
  }

  Vec3 focus_pos{0.0f, 0.0f, 0.0f};
  if (state.play_state == PlayState::Play &&
      player != rkg::ecs::kInvalidEntity && registry.get_transform(player)) {
    const auto* transform = registry.get_transform(player);
    focus_pos = {transform->position[0], transform->position[1], transform->position[2]};
  } else if (state.lock_editor_pivot) {
    focus_pos = {state.editor_pivot_world[0], state.editor_pivot_world[1], state.editor_pivot_world[2]};
  } else {
    rkg::ecs::Entity focus_entity = state.selected_entity;
    if (focus_entity == rkg::ecs::kInvalidEntity || !registry.get_transform(focus_entity)) {
      if (player != rkg::ecs::kInvalidEntity && registry.get_transform(player)) {
        focus_entity = player;
      } else if (registry.get_transform(state.fallback_entity)) {
        focus_entity = state.fallback_entity;
      }
    }
    if (const auto* transform = registry.get_transform(focus_entity)) {
      focus_pos = {transform->position[0], transform->position[1], transform->position[2]};
    }
  }
  if (state.play_state != PlayState::Play && !state.lock_editor_pivot) {
    state.editor_pivot_world[0] = focus_pos.x;
    state.editor_pivot_world[1] = focus_pos.y;
    state.editor_pivot_world[2] = focus_pos.z;
  }
  focus_pos = vec3_add(focus_pos, {state.camera_pan[0], state.camera_pan[1], state.camera_pan[2]});

  // Viewport camera controls:
  // - RMB drag: orbit (yaw/pitch)
  // - Mouse wheel: zoom (distance)
  // - MMB drag: pan (move pivot in view plane)
  const bool camera_input_enabled =
      state.viewport_focused && !state.chat_active && !io.WantTextInput;
  if (camera_input_enabled && ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
    state.camera_yaw += io.MouseDelta.x * 0.01f;
    state.camera_pitch += io.MouseDelta.y * 0.01f;
    if (state.camera_pitch > 1.4f) state.camera_pitch = 1.4f;
    if (state.camera_pitch < -1.4f) state.camera_pitch = -1.4f;
  }
  if (camera_input_enabled && std::abs(io.MouseWheel) > 0.0001f) {
    state.camera_distance -= io.MouseWheel * 0.4f;
    if (state.camera_distance < 1.5f) state.camera_distance = 1.5f;
    if (state.camera_distance > 12.0f) state.camera_distance = 12.0f;
  }

  const float cy = std::cos(state.camera_yaw);
  const float sy = std::sin(state.camera_yaw);
  const float cp = std::cos(state.camera_pitch);
  const float sp = std::sin(state.camera_pitch);
  const Vec3 forward = {cp * sy, sp, cp * cy};
  const Vec3 eye = vec3_sub(focus_pos, vec3_mul(forward, state.camera_distance));
  const Vec3 world_up = {0.0f, 1.0f, 0.0f};
  const Vec3 right = vec3_normalize(vec3_cross(world_up, forward));
  const Vec3 up = vec3_cross(forward, right);
  if (camera_input_enabled && ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
    const float pan_scale = state.camera_distance * 0.002f;
    const Vec3 pan_delta =
        vec3_add(vec3_mul(right, -io.MouseDelta.x * pan_scale), vec3_mul(up, io.MouseDelta.y * pan_scale));
    state.camera_pan[0] += pan_delta.x;
    state.camera_pan[1] += pan_delta.y;
    state.camera_pan[2] += pan_delta.z;
  }

  state.camera_eye[0] = eye.x;
  state.camera_eye[1] = eye.y;
  state.camera_eye[2] = eye.z;
  state.camera_forward[0] = forward.x;
  state.camera_forward[1] = forward.y;
  state.camera_forward[2] = forward.z;
  state.camera_right[0] = right.x;
  state.camera_right[1] = right.y;
  state.camera_right[2] = right.z;
  state.camera_up[0] = up.x;
  state.camera_up[1] = up.y;
  state.camera_up[2] = up.z;

  Mat4 view = mat4_look_at(eye, focus_pos, {0.0f, 1.0f, 0.0f});
  Mat4 proj = mat4_perspective(state.camera_fov, static_cast<float>(viewport_w) / viewport_h, 0.1f, 50.0f);
  Mat4 view_proj = mat4_mul(proj, view);
  rkg::set_vulkan_viewport_camera(view_proj.m);
  std::memcpy(state.camera_view_proj, view_proj.m, sizeof(state.camera_view_proj));

  float line_positions[rkg::VulkanViewportLineList::kMaxLines * 6]{};
  float line_colors[rkg::VulkanViewportLineList::kMaxLines * 4]{};
  uint32_t line_count = 0;
  auto add_line = [&](const Vec3& a, const Vec3& b, const float* color) {
    if (line_count >= rkg::VulkanViewportLineList::kMaxLines) {
      return;
    }
    const size_t base = static_cast<size_t>(line_count) * 6;
    line_positions[base + 0] = a.x;
    line_positions[base + 1] = a.y;
    line_positions[base + 2] = a.z;
    line_positions[base + 3] = b.x;
    line_positions[base + 4] = b.y;
    line_positions[base + 5] = b.z;
    const size_t cbase = static_cast<size_t>(line_count) * 4;
    line_colors[cbase + 0] = color[0];
    line_colors[cbase + 1] = color[1];
    line_colors[cbase + 2] = color[2];
    line_colors[cbase + 3] = color[3];
    line_count += 1;
  };

  const float world_grid_color[4] = {0.22f, 0.22f, 0.26f, 1.0f};
  const float character_grid_color[4] = {0.28f, 0.34f, 0.5f, 1.0f};
  const float axis_x_color[4] = {0.85f, 0.2f, 0.2f, 1.0f};
  const float axis_y_color[4] = {0.2f, 0.85f, 0.2f, 1.0f};
  const float axis_z_color[4] = {0.2f, 0.35f, 0.9f, 1.0f};
  const float skeleton_color[4] = {0.9f, 0.85f, 0.2f, 1.0f};

  if (state.show_world_grid) {
    const float extent = state.grid_half_extent;
    const int steps = static_cast<int>(std::floor(extent / state.grid_step));
    for (int i = -steps; i <= steps; ++i) {
      const float coord = static_cast<float>(i) * state.grid_step;
      add_line({-extent, 0.0f, coord}, {extent, 0.0f, coord}, world_grid_color);
      add_line({coord, 0.0f, -extent}, {coord, 0.0f, extent}, world_grid_color);
    }
  }

  if (state.show_character_grid) {
    const float extent = state.grid_half_extent * 0.5f;
    const float base_y = focus_pos.y;
    const int steps = static_cast<int>(std::floor(extent / state.grid_step));
    for (int i = -steps; i <= steps; ++i) {
      const float coord = static_cast<float>(i) * state.grid_step;
      add_line({focus_pos.x - extent, base_y, focus_pos.z + coord},
               {focus_pos.x + extent, base_y, focus_pos.z + coord}, character_grid_color);
      add_line({focus_pos.x + coord, base_y, focus_pos.z - extent},
               {focus_pos.x + coord, base_y, focus_pos.z + extent}, character_grid_color);
    }
  }

  if (state.show_world_axes) {
    const float axis_len = std::max(1.0f, state.grid_half_extent * 0.5f);
    add_line({0.0f, 0.0f, 0.0f}, {axis_len, 0.0f, 0.0f}, axis_x_color);
    add_line({0.0f, 0.0f, 0.0f}, {0.0f, axis_len, 0.0f}, axis_y_color);
    add_line({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, axis_len}, axis_z_color);
  }

  if (state.show_skeleton_debug) {
    for (auto& kv : registry.skeletons()) {
      const auto entity = kv.first;
      auto& skeleton = kv.second;
      const auto* transform = registry.get_transform(entity);
      if (!transform) {
        continue;
      }
      rkg::ecs::compute_skeleton_world_pose(*transform, skeleton);
      if (skeleton.world_pose.size() != skeleton.bones.size()) {
        continue;
      }
      const float joint_len = 0.06f;
      for (size_t i = 0; i < skeleton.bones.size(); ++i) {
        const auto& bone = skeleton.bones[i];
        const auto& world = skeleton.world_pose[i];
        const Vec3 pos{world.position[0], world.position[1], world.position[2]};
        if (bone.parent_index >= 0 &&
            static_cast<size_t>(bone.parent_index) < skeleton.world_pose.size()) {
          const auto& parent_world = skeleton.world_pose[bone.parent_index];
          const Vec3 parent_pos{parent_world.position[0], parent_world.position[1], parent_world.position[2]};
          add_line(parent_pos, pos, skeleton_color);
        }
        add_line({pos.x - joint_len, pos.y, pos.z}, {pos.x + joint_len, pos.y, pos.z}, skeleton_color);
        add_line({pos.x, pos.y - joint_len, pos.z}, {pos.x, pos.y + joint_len, pos.z}, skeleton_color);
        add_line({pos.x, pos.y, pos.z - joint_len}, {pos.x, pos.y, pos.z + joint_len}, skeleton_color);
      }
    }
  }

  if (player != rkg::ecs::kInvalidEntity) {
    const auto* transform = registry.get_transform(player);
    const auto* velocity = registry.get_velocity(player);
    if (transform && velocity) {
      const Vec3 pos{transform->position[0], transform->position[1], transform->position[2]};
      const Vec3 vel{velocity->linear[0], velocity->linear[1], velocity->linear[2]};
      const float vel_scale = 0.35f;
      const Vec3 tip = vec3_add(pos, vec3_mul(vel, vel_scale));
      const float vel_color[4] = {1.0f, 0.4f, 0.9f, 1.0f};
      add_line(pos, tip, vel_color);
    }
  }

  if (state.show_face_labels) {
    auto transform_point = [&](const Mat4& m, const Vec3& v) -> Vec3 {
      return {
          m.m[0] * v.x + m.m[4] * v.y + m.m[8] * v.z + m.m[12],
          m.m[1] * v.x + m.m[5] * v.y + m.m[9] * v.z + m.m[13],
          m.m[2] * v.x + m.m[6] * v.y + m.m[10] * v.z + m.m[14],
      };
    };
    auto transform_dir = [&](const Mat4& m, const Vec3& v) -> Vec3 {
      return {
          m.m[0] * v.x + m.m[4] * v.y + m.m[8] * v.z,
          m.m[1] * v.x + m.m[5] * v.y + m.m[9] * v.z,
          m.m[2] * v.x + m.m[6] * v.y + m.m[10] * v.z,
      };
    };

    struct GlyphSegment {
      float x1;
      float y1;
      float x2;
      float y2;
    };
    auto glyph_segments = [&](char c, const GlyphSegment*& segs, size_t& count) -> bool {
      switch (c) {
        case 'A': {
          static const GlyphSegment k[] = {{0,0,0,1},{1,0,1,1},{0,0.5f,1,0.5f}};
          segs = k; count = 3; return true;
        }
        case 'B': {
          static const GlyphSegment k[] = {{0,0,0,1},{0,1,1,1},{0,0.5f,1,0.5f},{0,0,1,0},{1,0,1,1}};
          segs = k; count = 5; return true;
        }
        case 'C': {
          static const GlyphSegment k[] = {{0,0,0,1},{0,1,1,1},{0,0,1,0}};
          segs = k; count = 3; return true;
        }
        case 'E': {
          static const GlyphSegment k[] = {{0,0,0,1},{0,1,1,1},{0,0.5f,1,0.5f},{0,0,1,0}};
          segs = k; count = 4; return true;
        }
        case 'F': {
          static const GlyphSegment k[] = {{0,0,0,1},{0,1,1,1},{0,0.5f,1,0.5f}};
          segs = k; count = 3; return true;
        }
        case 'G': {
          static const GlyphSegment k[] = {{0,0,0,1},{0,1,1,1},{0,0,1,0},{1,0,1,0.5f},{0.5f,0.5f,1,0.5f}};
          segs = k; count = 5; return true;
        }
        case 'H': {
          static const GlyphSegment k[] = {{0,0,0,1},{1,0,1,1},{0,0.5f,1,0.5f}};
          segs = k; count = 3; return true;
        }
        case 'I': {
          static const GlyphSegment k[] = {{0,1,1,1},{0.5f,0,0.5f,1},{0,0,1,0}};
          segs = k; count = 3; return true;
        }
        case 'K': {
          static const GlyphSegment k[] = {{0,0,0,1},{1,1,0,0.5f},{0,0.5f,1,0}};
          segs = k; count = 3; return true;
        }
        case 'L': {
          static const GlyphSegment k[] = {{0,0,0,1},{0,0,1,0}};
          segs = k; count = 2; return true;
        }
        case 'M': {
          static const GlyphSegment k[] = {{0,0,0,1},{1,0,1,1},{0,1,0.5f,0},{0.5f,0,1,1}};
          segs = k; count = 4; return true;
        }
        case 'N': {
          static const GlyphSegment k[] = {{0,0,0,1},{1,0,1,1},{0,0,1,1}};
          segs = k; count = 3; return true;
        }
        case 'O': {
          static const GlyphSegment k[] = {{0,0,0,1},{1,0,1,1},{0,1,1,1},{0,0,1,0}};
          segs = k; count = 4; return true;
        }
        case 'P': {
          static const GlyphSegment k[] = {{0,0,0,1},{0,1,1,1},{0,0.5f,1,0.5f},{1,0.5f,1,1}};
          segs = k; count = 4; return true;
        }
        case 'R': {
          static const GlyphSegment k[] = {{0,0,0,1},{0,1,1,1},{0,0.5f,1,0.5f},{1,0.5f,1,1},{0,0.5f,1,0}};
          segs = k; count = 5; return true;
        }
        case 'T': {
          static const GlyphSegment k[] = {{0,1,1,1},{0.5f,0,0.5f,1}};
          segs = k; count = 2; return true;
        }
        default:
          return false;
      }
    };

    struct Face {
      Vec3 normal;
      Vec3 u;
      Vec3 v;
      const char* word;
    };
    const Face faces[] = {
        {{0,0,1}, {1,0,0}, {0,1,0}, "FRONT"},
        {{0,0,-1}, {-1,0,0}, {0,1,0}, "BACK"},
        {{1,0,0}, {0,0,-1}, {0,1,0}, "LEFT"},
        {{-1,0,0}, {0,0,1}, {0,1,0}, "RIGHT"},
        {{0,1,0}, {1,0,0}, {0,0,1}, "TOP"},
        {{0,-1,0}, {1,0,0}, {0,0,-1}, "BOTTOM"},
    };

    rkg::ecs::Entity label_entity = player;
    if (label_entity == rkg::ecs::kInvalidEntity || !registry.get_transform(label_entity)) {
      label_entity = state.selected_entity;
    }
    if (label_entity != rkg::ecs::kInvalidEntity) {
      if (const auto* transform = registry.get_transform(label_entity)) {
        const Vec3 pos = {transform->position[0], transform->position[1], transform->position[2]};
        const Vec3 rot = {transform->rotation[0], transform->rotation[1], transform->rotation[2]};
        const Vec3 scl = {transform->scale[0], transform->scale[1], transform->scale[2]};
        const Mat4 rot_m = mat4_rotation_xyz(rot);
        const Mat4 model = mat4_mul(mat4_translation(pos), mat4_mul(rot_m, mat4_scale(scl)));
        const float spacing = 0.2f;
        const float face_half = 0.5f;
        const float normal_offset = 0.02f;
        const float text_color[4] = {0.0f, 0.0f, 0.0f, 1.0f};

        for (const auto& face : faces) {
          const Vec3 local_center = vec3_mul(face.normal, face_half);
          const Vec3 world_center = transform_point(model, local_center);
          const int letter_count = static_cast<int>(std::strlen(face.word));
          if (letter_count <= 0) {
            continue;
          }
          const float size = 0.7f / (letter_count + spacing * (letter_count - 1));
          const float word_w = size * (letter_count + spacing * (letter_count - 1));
          const float word_h = size;
          const Vec3 origin = vec3_add(local_center,
                                       vec3_add(vec3_mul(face.u, -word_w * 0.5f),
                                                vec3_mul(face.v, -word_h * 0.5f)));
          for (int i = 0; i < letter_count; ++i) {
            const char c = face.word[i];
            const GlyphSegment* segs = nullptr;
            size_t seg_count = 0;
            if (!glyph_segments(c, segs, seg_count)) {
              continue;
            }
            const float base_x = i * size * (1.0f + spacing);
            for (size_t s = 0; s < seg_count; ++s) {
              const auto& seg = segs[s];
              const Vec3 p0_local = vec3_add(origin,
                                             vec3_add(vec3_mul(face.u, (base_x + seg.x1 * size)),
                                                      vec3_add(vec3_mul(face.v, seg.y1 * size),
                                                               vec3_mul(face.normal, normal_offset))));
              const Vec3 p1_local = vec3_add(origin,
                                             vec3_add(vec3_mul(face.u, (base_x + seg.x2 * size)),
                                                      vec3_add(vec3_mul(face.v, seg.y2 * size),
                                                               vec3_mul(face.normal, normal_offset))));
              const Vec3 p0 = transform_point(model, p0_local);
              const Vec3 p1 = transform_point(model, p1_local);
              add_line(p0, p1, text_color);
            }
          }
        }
      }
    }
  }

  if (line_count > 0) {
    rkg::set_vulkan_viewport_line_list(line_positions, line_colors, line_count);
  } else {
    rkg::set_vulkan_viewport_line_list(nullptr, nullptr, 0);
  }

  if (state.pick_requested) {
    state.pick_requested = false;
    const float local_x = state.pick_mouse_pos[0] - state.viewport_pos[0];
    const float local_y = state.pick_mouse_pos[1] - state.viewport_pos[1];
    if (viewport_w > 0 && viewport_h > 0 &&
        local_x >= 0.0f && local_y >= 0.0f &&
        local_x <= static_cast<float>(viewport_w) &&
        local_y <= static_cast<float>(viewport_h)) {
      const float ndc_x = (local_x / static_cast<float>(viewport_w)) * 2.0f - 1.0f;
      const float ndc_y = 1.0f - (local_y / static_cast<float>(viewport_h)) * 2.0f;
      const float fov_rad = state.camera_fov * 3.14159265f / 180.0f;
      const float tan_half = std::tan(fov_rad * 0.5f);
      const float aspect = static_cast<float>(viewport_w) / static_cast<float>(viewport_h);
      const Vec3 dir = vec3_normalize(vec3_add(
          vec3_add(vec3_mul(right, ndc_x * aspect * tan_half), vec3_mul(up, ndc_y * tan_half)),
          forward));
      const Ray ray{eye, dir};

      float best_t = std::numeric_limits<float>::max();
      rkg::ecs::Entity best = rkg::ecs::kInvalidEntity;
      for (const auto entity : registry.entities()) {
        const auto* transform = registry.get_transform(entity);
        const auto* renderable = registry.get_renderable(entity);
        if (!transform || !renderable) continue;
        const float sx = std::abs(transform->scale[0]);
        const float sy = std::abs(transform->scale[1]);
        const float sz = std::abs(transform->scale[2]);
        Vec3 half = {0.5f * sx, 0.5f * sy, 0.5f * sz};
        if (renderable->mesh == rkg::ecs::MeshId::Quad) {
          half.z = std::max(half.z, 0.01f);
        }
        const Vec3 pos = {transform->position[0], transform->position[1], transform->position[2]};
        const Vec3 min = vec3_sub(pos, half);
        const Vec3 max = vec3_add(pos, half);
        float t = 0.0f;
        if (ray_intersect_aabb(ray, min, max, t) && t < best_t) {
          best_t = t;
          best = entity;
        }
      }
      set_selected_entity(state, best);
      state.auto_select = false;
    }
  }

  float mvps[rkg::VulkanViewportDrawList::kMaxInstances * 16]{};
  float colors[rkg::VulkanViewportDrawList::kMaxInstances * 4]{};
  uint32_t mesh_ids[rkg::VulkanViewportDrawList::kMaxInstances]{};
  uint32_t instance_count = 0;
  for (const auto entity : registry.entities()) {
    if (instance_count >= rkg::VulkanViewportDrawList::kMaxInstances) {
      break;
    }
    const auto* transform = registry.get_transform(entity);
    const auto* renderable = registry.get_renderable(entity);
    if (!transform || !renderable) {
      continue;
    }
    const Vec3 pos = {transform->position[0], transform->position[1], transform->position[2]};
    const Vec3 rot = {transform->rotation[0], transform->rotation[1], transform->rotation[2]};
    const Vec3 scl = {transform->scale[0], transform->scale[1], transform->scale[2]};
    Mat4 model = mat4_mul(mat4_translation(pos), mat4_mul(mat4_rotation_xyz(rot), mat4_scale(scl)));
    Mat4 mvp = mat4_mul(proj, mat4_mul(view, model));
    std::memcpy(mvps + instance_count * 16, mvp.m, sizeof(float) * 16);
    mesh_ids[instance_count] = static_cast<uint32_t>(renderable->mesh);
    const size_t color_base = static_cast<size_t>(instance_count) * 4;
    float r = renderable->color[0];
    float g = renderable->color[1];
    float b = renderable->color[2];
    float a = renderable->color[3];
    if (entity == state.selected_entity) {
      r = std::min(1.0f, r * 0.4f + 0.6f);
      g = std::min(1.0f, g * 0.4f + 0.6f);
      b = b * 0.4f;
    }
    colors[color_base + 0] = r;
    colors[color_base + 1] = g;
    colors[color_base + 2] = b;
    colors[color_base + 3] = a;
    instance_count += 1;
  }

  if (instance_count > 0) {
    rkg::set_vulkan_viewport_draw_list(mvps, mesh_ids, colors, instance_count);
  } else {
    rkg::set_vulkan_viewport_draw_list(nullptr, nullptr, nullptr, 0);
  }
}

void draw_editor_ui(void* user_data) {
  auto* state = static_cast<EditorState*>(user_data);
  if (!state || !state->runtime) return;

  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->WorkPos);
  ImGui::SetNextWindowSize(viewport->WorkSize);
  ImGui::SetNextWindowViewport(viewport->ID);

  ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
                                  ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                                  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                  ImGuiWindowFlags_NoNavFocus;
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::Begin("EditorDockspace", nullptr, window_flags);
  ImGui::PopStyleVar(2);

  ImGuiID dockspace_id = ImGui::GetID("EditorDockspace");
  ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode;
  ImGui::DockSpace(dockspace_id, ImVec2(0, 0), dockspace_flags);
  if (!state->dock_built) {
    build_dock_layout(*state);
  }
  ImGui::End();

  draw_toolbar(*state);
  draw_viewport(*state);
  draw_scene_panel(*state);
  draw_inspector_panel(*state);
  draw_content_panel(*state);
  draw_diff_preview_panel(*state);
  draw_chat_panel(*state);
  draw_runs_browser_panel(*state);

  handle_undo_redo(*state);

  ImGuiIO& io = ImGui::GetIO();
  state->ui_capturing = io.WantCaptureKeyboard || io.WantCaptureMouse || io.WantTextInput || state->chat_active;
}
#endif

} // namespace

int main(int argc, char** argv) {
  std::optional<fs::path> project_override;
  bool disable_ui = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--project" && i + 1 < argc) {
      project_override = fs::path(argv[++i]);
    } else if (arg == "--no-ui" || arg == "--disable-debug-ui") {
      disable_ui = true;
    }
  }

  if (!project_override.has_value()) {
    const auto paths = rkg::resolve_paths(argc > 0 ? argv[0] : nullptr, std::nullopt, "demo_game");
    const fs::path blank_root = paths.root / "build" / "blank_project";
    std::error_code ec;
    fs::create_directories(blank_root / "content" / "levels", ec);
    const fs::path project_path = blank_root / "project.yaml";
    if (!fs::exists(project_path)) {
      std::ofstream out(project_path);
      out << "project:\n";
      out << "  name: blank_project\n";
      out << "  renderer: vulkan\n";
      out << "  plugins:\n";
      out << "    - debug_ui_imgui\n";
      out << "  initial_level: content/levels/blank.yaml\n";
      out << "  dev_mode: true\n";
    }
    const fs::path level_path = blank_root / "content" / "levels" / "blank.yaml";
    if (!fs::exists(level_path)) {
      std::ofstream level_out(level_path);
      level_out << "name: blank\n";
      level_out << "entities: []\n";
    }
    project_override = blank_root;
  }

  rkg::runtime::RuntimeHost runtime;
  rkg::runtime::RuntimeHostInit init;
  init.argv0 = argc > 0 ? argv[0] : nullptr;
  init.project_override = project_override;
  init.default_project = "demo_game";
  init.app_name = "rkg_editor";
  init.window = {1600, 900, "rkg_editor"};
  init.force_debug_ui = !disable_ui;
  init.disable_debug_ui = disable_ui;

  std::string error;
  if (!runtime.init(init, error)) {
    rkg::log::error(error.empty() ? "rkg_editor init failed" : error);
    return 1;
  }

#if RKG_EDITOR_IMGUI
  const bool use_ui = !disable_ui;
  EditorState state;
  state.runtime = &runtime;
  if (use_ui) {
    rkg::debug_ui::set_show_builtin(false);
    rkg::debug_ui::set_docking_enabled(true);

    state.agent.openai_available = openai_available(state.agent.openai_error);
    state.overrides.path = runtime.project_root() / "editor_overrides.yaml";
    sync_overrides_state(state);

    rkg::debug_ui::set_draw_callback(&draw_editor_ui, &state);
  } else {
    rkg::log::warn("debug_ui disabled via --no-ui");
  }
#else
  rkg::log::error("rkg_editor requires ImGui + Vulkan (RKG_ENABLE_IMGUI && RKG_ENABLE_VULKAN)");
  runtime.shutdown();
  return 1;
#endif

  while (!runtime.platform().should_quit()) {
    runtime.platform().poll_events();
    const float frame_dt = runtime.platform().delta_seconds();

#if RKG_EDITOR_IMGUI
    runtime.update_input();
    if (use_ui) {
      update_agent_state(state);
      update_content_state(state);
      update_runs_browser_state(state);
      sync_overrides_state(state);
      update_camera_and_draw_list(state);

      if (state.stop_requested) {
        runtime.force_reload("stop");
        state.stop_requested = false;
        state.selected_entity = rkg::ecs::kInvalidEntity;
        state.selected_name.clear();
        state.auto_select = true;
        state.fallback_entity = rkg::ecs::kInvalidEntity;
      }
    }

    float sim_dt = 0.0f;
    bool run_sim = false;

    if (state.play_state == PlayState::Play) {
      sim_dt = frame_dt;
      run_sim = true;
    } else if (state.step_requested) {
      sim_dt = state.fixed_step;
      run_sim = true;
      state.step_requested = false;
    }

    rkg::runtime::FrameParams params;
    params.frame_dt = frame_dt;
    params.sim_dt = sim_dt;
    params.run_simulation = run_sim;
    params.update_input = false;
    auto action_provider = [&](const std::string& name) -> rkg::input::ActionState {
      if (!state.viewport_focused || state.chat_active) {
        return {};
      }
      return runtime.input_action(name);
    };
    runtime.tick(params, action_provider);
#else
    (void)frame_dt;
    runtime.platform().request_quit();
#endif
  }

  runtime.shutdown();
  return 0;
}
