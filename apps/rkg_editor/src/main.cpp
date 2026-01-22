#include "rkg/runtime_host.h"

#include "subprocess_runner.h"

#include "rkg/log.h"
#include "rkg/renderer_hooks.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <cstring>
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

namespace fs = std::filesystem;

namespace {

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
  std::string last_error;
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
  float camera_yaw = 0.0f;
  float camera_pitch = 0.3f;
  float camera_distance = 4.0f;
  float camera_fov = 60.0f;

  rkg::ecs::Entity selected_entity = rkg::ecs::kInvalidEntity;
  std::string selected_name;
  rkg::ecs::Entity fallback_entity = rkg::ecs::kInvalidEntity;

  AgentPanelState agent;
  ContentPanelState content;
};

std::string join_lines(const std::vector<std::string>& lines) {
  std::ostringstream out;
  for (size_t i = 0; i < lines.size(); ++i) {
    out << lines[i];
    if (i + 1 < lines.size()) out << "\n";
  }
  return out.str();
}

struct Vec3 {
  float x;
  float y;
  float z;
};

struct Mat4 {
  float m[16];
};

Mat4 mat4_identity() {
  Mat4 out{};
  out.m[0] = 1.0f;
  out.m[5] = 1.0f;
  out.m[10] = 1.0f;
  out.m[15] = 1.0f;
  return out;
}

Mat4 mat4_mul(const Mat4& a, const Mat4& b) {
  Mat4 out{};
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      out.m[col * 4 + row] =
          a.m[0 * 4 + row] * b.m[col * 4 + 0] +
          a.m[1 * 4 + row] * b.m[col * 4 + 1] +
          a.m[2 * 4 + row] * b.m[col * 4 + 2] +
          a.m[3 * 4 + row] * b.m[col * 4 + 3];
    }
  }
  return out;
}

Vec3 vec3_sub(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 vec3_mul(const Vec3& a, float s) {
  return {a.x * s, a.y * s, a.z * s};
}

float vec3_dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 vec3_cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vec3 vec3_normalize(const Vec3& v) {
  const float len = std::sqrt(vec3_dot(v, v));
  if (len <= 0.00001f) {
    return {0.0f, 0.0f, 0.0f};
  }
  const float inv = 1.0f / len;
  return {v.x * inv, v.y * inv, v.z * inv};
}

Mat4 mat4_look_at(const Vec3& eye, const Vec3& target, const Vec3& up) {
  const Vec3 f = vec3_normalize(vec3_sub(target, eye));
  const Vec3 s = vec3_normalize(vec3_cross(f, up));
  const Vec3 u = vec3_cross(s, f);

  Mat4 out = mat4_identity();
  out.m[0] = s.x;
  out.m[4] = s.y;
  out.m[8] = s.z;
  out.m[1] = u.x;
  out.m[5] = u.y;
  out.m[9] = u.z;
  out.m[2] = -f.x;
  out.m[6] = -f.y;
  out.m[10] = -f.z;
  out.m[12] = -vec3_dot(s, eye);
  out.m[13] = -vec3_dot(u, eye);
  out.m[14] = vec3_dot(f, eye);
  return out;
}

Mat4 mat4_perspective(float fov_deg, float aspect, float znear, float zfar) {
  Mat4 out{};
  const float fov_rad = fov_deg * 3.14159265f / 180.0f;
  const float f = 1.0f / std::tan(fov_rad * 0.5f);
  out.m[0] = f / aspect;
  out.m[5] = -f; // flip Y for Vulkan NDC
  out.m[10] = zfar / (znear - zfar);
  out.m[11] = -1.0f;
  out.m[14] = (zfar * znear) / (znear - zfar);
  return out;
}

Mat4 mat4_translation(const Vec3& t) {
  Mat4 out = mat4_identity();
  out.m[12] = t.x;
  out.m[13] = t.y;
  out.m[14] = t.z;
  return out;
}

Mat4 mat4_scale(const Vec3& s) {
  Mat4 out = mat4_identity();
  out.m[0] = s.x;
  out.m[5] = s.y;
  out.m[10] = s.z;
  return out;
}

Mat4 mat4_rotation_x(float r) {
  Mat4 out = mat4_identity();
  const float c = std::cos(r);
  const float s = std::sin(r);
  out.m[5] = c;
  out.m[6] = s;
  out.m[9] = -s;
  out.m[10] = c;
  return out;
}

Mat4 mat4_rotation_y(float r) {
  Mat4 out = mat4_identity();
  const float c = std::cos(r);
  const float s = std::sin(r);
  out.m[0] = c;
  out.m[2] = -s;
  out.m[8] = s;
  out.m[10] = c;
  return out;
}

Mat4 mat4_rotation_z(float r) {
  Mat4 out = mat4_identity();
  const float c = std::cos(r);
  const float s = std::sin(r);
  out.m[0] = c;
  out.m[1] = s;
  out.m[4] = -s;
  out.m[5] = c;
  return out;
}

Mat4 mat4_rotation_xyz(const Vec3& r) {
  return mat4_mul(mat4_rotation_z(r.z), mat4_mul(mat4_rotation_y(r.y), mat4_rotation_x(r.x)));
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

void update_content_state(EditorState& state) {
  state.content.cook_process.update();
  state.content.watch_process.update();
}

#if RKG_EDITOR_IMGUI
void draw_copy_button(const char* id, const std::string& value) {
  ImGui::SameLine();
  if (ImGui::SmallButton(id)) {
    ImGui::SetClipboardText(value.c_str());
  }
}

void build_dock_layout(EditorState& state) {
  ImGuiID dockspace_id = ImGui::GetID("EditorDockspace");
  ImGui::DockBuilderRemoveNode(dockspace_id);
  ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace | ImGuiDockNodeFlags_PassthruCentralNode);
  ImGui::DockBuilderSetNodeSize(dockspace_id, ImGui::GetMainViewport()->Size);

  ImGuiID dock_main = dockspace_id;
  ImGuiID dock_left = ImGui::DockBuilderSplitNode(dock_main, ImGuiDir_Left, 0.22f, nullptr, &dock_main);
  ImGuiID dock_right = ImGui::DockBuilderSplitNode(dock_main, ImGuiDir_Right, 0.28f, nullptr, &dock_main);
  ImGuiID dock_bottom = ImGui::DockBuilderSplitNode(dock_main, ImGuiDir_Down, 0.25f, nullptr, &dock_main);
  ImGuiID dock_top = ImGui::DockBuilderSplitNode(dock_main, ImGuiDir_Up, 0.08f, nullptr, &dock_main);

  ImGuiID dock_left_bottom = ImGui::DockBuilderSplitNode(dock_left, ImGuiDir_Down, 0.5f, nullptr, &dock_left);
  ImGuiID dock_right_bottom = ImGui::DockBuilderSplitNode(dock_right, ImGuiDir_Down, 0.4f, nullptr, &dock_right);

  ImGui::DockBuilderDockWindow("Viewport", dock_main);
  ImGui::DockBuilderDockWindow("Toolbar", dock_top);
  ImGui::DockBuilderDockWindow("Scene", dock_left);
  ImGui::DockBuilderDockWindow("Inspector", dock_left_bottom);
  ImGui::DockBuilderDockWindow("Content", dock_bottom);
  ImGui::DockBuilderDockWindow("Chat", dock_right);
  ImGui::DockBuilderDockWindow("Agent Runs", dock_right_bottom);

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
  }

  ImGui::SameLine();
  ImGui::Text("Mode: %s | dt %.2f ms | %.1f fps", label, dt * 1000.0f, fps);

  ImGui::End();
}

void draw_viewport(EditorState& state) {
  ImGui::Begin("Viewport", nullptr,
               ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
                   ImGuiWindowFlags_NoCollapse);
  const ImVec2 avail = ImGui::GetContentRegionAvail();
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

  const bool hovered = ImGui::IsWindowHovered();
  if (hovered && ImGui::IsMouseClicked(0)) {
    state.viewport_focused = true;
  }
  if (!hovered && ImGui::IsMouseClicked(0)) {
    state.viewport_focused = false;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
    state.viewport_focused = false;
  }
  ImGui::Text("Focus: %s", state.viewport_focused ? "Viewport" : "UI");
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
    state.selected_entity = rkg::ecs::kInvalidEntity;
    state.selected_name.clear();
  }

  if (state.selected_entity == rkg::ecs::kInvalidEntity && !entries.empty()) {
    state.selected_entity = entries.front().second;
    state.selected_name = entries.front().first;
  }

  if (entries.empty()) {
    ImGui::TextUnformatted("No entities.");
  } else {
    for (const auto& entry : entries) {
      const bool selected = (entry.second == state.selected_entity);
      if (ImGui::Selectable(entry.first.c_str(), selected)) {
        state.selected_entity = entry.second;
        state.selected_name = entry.first;
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
  ImGui::Text("Entity: %u", entity);
  if (!state.selected_name.empty()) {
    ImGui::Text("Name: %s", state.selected_name.c_str());
  }
  if (const auto* transform = state.runtime->registry().get_transform(entity)) {
    ImGui::Separator();
    ImGui::Text("Transform");
    float pos[3] = {transform->position[0], transform->position[1], transform->position[2]};
    if (ImGui::DragFloat3("Position", pos, 0.05f)) {
      auto* writable = state.runtime->registry().get_transform(entity);
      if (writable) {
        writable->position[0] = pos[0];
        writable->position[1] = pos[1];
        writable->position[2] = pos[2];
      }
    }
    ImGui::Text("Rotation: %.2f %.2f %.2f", transform->rotation[0], transform->rotation[1], transform->rotation[2]);
    ImGui::Text("Scale: %.2f %.2f %.2f", transform->scale[0], transform->scale[1], transform->scale[2]);
  } else {
    ImGui::TextUnformatted("Transform: (none)");
  }

  auto* renderable = state.runtime->registry().get_renderable(entity);
  ImGui::Separator();
  ImGui::Text("Renderable");
  if (!renderable) {
    if (ImGui::Button("Add Renderable")) {
      rkg::ecs::Renderable fresh{};
      state.runtime->registry().set_renderable(entity, fresh);
    }
  } else {
    const char* mesh_items[] = {"cube", "quad"};
    int mesh_index = 0;
    if (renderable->mesh == rkg::ecs::MeshId::Quad) {
      mesh_index = 1;
    }
    if (ImGui::Combo("Mesh", &mesh_index, mesh_items, 2)) {
      renderable->mesh = (mesh_index == 1) ? rkg::ecs::MeshId::Quad : rkg::ecs::MeshId::Cube;
    }
    float color[4] = {renderable->color[0], renderable->color[1], renderable->color[2], renderable->color[3]};
    if (ImGui::ColorEdit4("Color", color)) {
      renderable->color[0] = color[0];
      renderable->color[1] = color[1];
      renderable->color[2] = color[2];
      renderable->color[3] = color[3];
    }
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

void draw_agent_runs_panel(EditorState& state) {
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

void update_camera_and_draw_list(EditorState& state) {
  int viewport_w = 0;
  int viewport_h = 0;
  rkg::debug_ui::viewport_size(&viewport_w, &viewport_h);
  if (viewport_w <= 0 || viewport_h <= 0) {
    return;
  }

  const ImGuiIO& io = ImGui::GetIO();
  const bool can_control = state.viewport_focused && !state.ui_capturing;
  auto& registry = state.runtime->registry();

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
        if (state.runtime->input_action("MoveForward").held) transform->position[1] += speed;
        if (state.runtime->input_action("MoveBack").held) transform->position[1] -= speed;
        if (state.runtime->input_action("MoveLeft").held) transform->position[0] -= speed;
        if (state.runtime->input_action("MoveRight").held) transform->position[0] += speed;
      }
    }
  }

  rkg::ecs::Entity focus_entity = state.selected_entity;
  if (focus_entity == rkg::ecs::kInvalidEntity || !registry.get_transform(focus_entity)) {
    if (player != rkg::ecs::kInvalidEntity && registry.get_transform(player)) {
      focus_entity = player;
    } else if (registry.get_transform(state.fallback_entity)) {
      focus_entity = state.fallback_entity;
    }
  }

  Vec3 focus_pos{0.0f, 0.0f, 0.0f};
  if (const auto* transform = registry.get_transform(focus_entity)) {
    focus_pos = {transform->position[0], transform->position[1], transform->position[2]};
  }

  if (state.play_state == PlayState::Edit || state.play_state == PlayState::Pause) {
    if (can_control && ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
      state.camera_yaw += io.MouseDelta.x * 0.01f;
      state.camera_pitch += io.MouseDelta.y * 0.01f;
      if (state.camera_pitch > 1.4f) state.camera_pitch = 1.4f;
      if (state.camera_pitch < -1.4f) state.camera_pitch = -1.4f;
    }
    if (can_control && std::abs(io.MouseWheel) > 0.0001f) {
      state.camera_distance -= io.MouseWheel * 0.4f;
      if (state.camera_distance < 1.5f) state.camera_distance = 1.5f;
      if (state.camera_distance > 12.0f) state.camera_distance = 12.0f;
    }
  }

  const float cy = std::cos(state.camera_yaw);
  const float sy = std::sin(state.camera_yaw);
  const float cp = std::cos(state.camera_pitch);
  const float sp = std::sin(state.camera_pitch);
  const Vec3 forward = {cp * sy, sp, cp * cy};
  const Vec3 eye = vec3_sub(focus_pos, vec3_mul(forward, state.camera_distance));

  Mat4 view = mat4_look_at(eye, focus_pos, {0.0f, 1.0f, 0.0f});
  Mat4 proj = mat4_perspective(state.camera_fov, static_cast<float>(viewport_w) / viewport_h, 0.1f, 50.0f);

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
    colors[color_base + 0] = renderable->color[0];
    colors[color_base + 1] = renderable->color[1];
    colors[color_base + 2] = renderable->color[2];
    colors[color_base + 3] = renderable->color[3];
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
  draw_chat_panel(*state);
  draw_agent_runs_panel(*state);

  ImGuiIO& io = ImGui::GetIO();
  state->ui_capturing = io.WantCaptureKeyboard || io.WantCaptureMouse || io.WantTextInput || state->chat_active;
}
#endif

} // namespace

int main(int argc, char** argv) {
  std::optional<fs::path> project_override;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--project" && i + 1 < argc) {
      project_override = fs::path(argv[++i]);
    }
  }

  rkg::runtime::RuntimeHost runtime;
  rkg::runtime::RuntimeHostInit init;
  init.argv0 = argc > 0 ? argv[0] : nullptr;
  init.project_override = project_override;
  init.default_project = "demo_game";
  init.app_name = "rkg_editor";
  init.window = {1600, 900, "rkg_editor"};
  init.force_debug_ui = true;

  std::string error;
  if (!runtime.init(init, error)) {
    rkg::log::error(error.empty() ? "rkg_editor init failed" : error);
    return 1;
  }

#if RKG_EDITOR_IMGUI
  rkg::debug_ui::set_show_builtin(false);
  rkg::debug_ui::set_docking_enabled(true);

  EditorState state;
  state.runtime = &runtime;
  state.agent.openai_available = openai_available(state.agent.openai_error);

  rkg::debug_ui::set_draw_callback(&draw_editor_ui, &state);
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
    update_agent_state(state);
    update_content_state(state);
    update_camera_and_draw_list(state);

    if (state.stop_requested) {
      runtime.force_reload("stop");
      state.stop_requested = false;
      state.selected_entity = rkg::ecs::kInvalidEntity;
      state.selected_name.clear();
      state.fallback_entity = rkg::ecs::kInvalidEntity;
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
      if (!state.viewport_focused || state.ui_capturing) {
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
