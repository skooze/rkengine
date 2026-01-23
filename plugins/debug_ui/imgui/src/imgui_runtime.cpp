#include "rkg_debug_ui/imgui_api.h"

#include "rkg/ai_results.h"
#include "rkg/ecs.h"
#include "rkg/host_context.h"
#include "rkg/log.h"
#include "rkg/renderer_hooks.h"

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

#include <imgui.h>
#include <backends/imgui_impl_sdl3.h>
#include <backends/imgui_impl_vulkan.h>

#include <SDL3/SDL.h>

#include <array>
#include <chrono>
#include <fstream>
#include <optional>
#include <type_traits>

namespace rkg::debug_ui {

namespace {
struct DebugUiState {
  bool initialized = false;
  bool visible = true;
  bool has_draw_data = false;
  bool render_inside_pass = true;
  bool force_reload = false;
  bool show_builtin = true;
  bool docking_enabled = false;
  rkg::debug_ui::DrawCallback draw_callback = nullptr;
  void* draw_user = nullptr;
  rkg::HostContext* host = nullptr;
  std::filesystem::path root;
  std::string renderer_name = "Unknown";

  VkInstance instance = VK_NULL_HANDLE;
  VkPhysicalDevice physical = VK_NULL_HANDLE;
  VkDevice device = VK_NULL_HANDLE;
  VkQueue queue = VK_NULL_HANDLE;
  VkRenderPass render_pass = VK_NULL_HANDLE;
  VkFormat swapchain_format = VK_FORMAT_UNDEFINED;
  uint32_t queue_family = 0;
  uint32_t image_count = 2;
  SDL_Window* window = nullptr;
  VkDescriptorPool descriptor_pool = VK_NULL_HANDLE;
  VkCommandPool font_cmd_pool = VK_NULL_HANDLE;

  float fps = 0.0f;
  std::string last_reload_time = "N/A";
  std::string last_reload_error = "";
  rkg::AiResultsSummary ai_results{};
  std::string ai_error = "no results";
  std::string ai_goal = "";
  std::string ai_provider = "";
  std::string ai_model = "";
  bool ai_valid = false;
  uint64_t ai_mtime = 0;
  uint64_t ai_info_mtime = 0;

  void* viewport_texture = nullptr;
  uint32_t viewport_width = 0;
  uint32_t viewport_height = 0;
  uint64_t viewport_version = 0;
  bool viewport_available = false;
  std::string viewport_error;
} g_state;

bool create_descriptor_pool() {
  VkDescriptorPoolSize pool_sizes[] = {
      {VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
      {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
      {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
      {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000},
  };

  VkDescriptorPoolCreateInfo pool_info{};
  pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
  pool_info.maxSets = 1000 * static_cast<uint32_t>(std::size(pool_sizes));
  pool_info.poolSizeCount = static_cast<uint32_t>(std::size(pool_sizes));
  pool_info.pPoolSizes = pool_sizes;
  return vkCreateDescriptorPool(g_state.device, &pool_info, nullptr, &g_state.descriptor_pool) == VK_SUCCESS;
}

template <typename T>
constexpr bool has_render_pass_member() {
  return requires(T value) { value.RenderPass; };
}

template <typename T>
constexpr bool has_dynamic_rendering_member() {
  return requires(T value) { value.UseDynamicRendering; };
}

template <typename T>
constexpr bool has_color_format_member() {
  return requires(T value) { value.ColorAttachmentFormat; };
}

template <typename T>
constexpr bool has_msaa_member() {
  return requires(T value) { value.MSAASamples; };
}

template <typename T>
constexpr bool has_pipeline_rendering_member() {
  return requires(T value) { value.PipelineRenderingCreateInfo; };
}

template <typename T>
void set_render_pass(T& info, VkRenderPass pass) {
  if constexpr (has_render_pass_member<T>()) {
    info.RenderPass = pass;
  }
}

template <typename T>
void set_dynamic_rendering(T& info, bool enable) {
  if constexpr (has_dynamic_rendering_member<T>()) {
    info.UseDynamicRendering = enable;
  }
}

template <typename T>
void set_color_format(T& info, VkFormat format) {
  if constexpr (has_color_format_member<T>()) {
    info.ColorAttachmentFormat = format;
  }
}

template <typename T>
void set_msaa_samples(T& info, VkSampleCountFlagBits samples) {
  if constexpr (has_msaa_member<T>()) {
    info.MSAASamples = samples;
  }
}

template <typename T>
void set_pipeline_rendering_info(T& info, VkPipelineRenderingCreateInfo* rendering_info) {
  if constexpr (has_pipeline_rendering_member<T>()) {
    info.PipelineRenderingCreateInfo = rendering_info;
  }
}

static bool texture_id_is_empty(ImTextureID tex_id) {
  return tex_id == static_cast<ImTextureID>(0);
}

template <typename T>
static std::optional<ImTextureID> draw_cmd_texture_id(const T& cmd) {
  if constexpr (requires { cmd.TextureId; }) {
    return cmd.TextureId;
  } else if constexpr (requires { cmd.TexID; }) {
    return cmd.TexID;
  } else {
    return std::nullopt;
  }
}

static bool draw_data_has_empty_texture(const ImDrawData* draw_data) {
  if (!draw_data) {
    return true;
  }
  for (int list_index = 0; list_index < draw_data->CmdListsCount; ++list_index) {
    const ImDrawList* list = draw_data->CmdLists[list_index];
    if (!list) {
      continue;
    }
    for (int cmd_index = 0; cmd_index < list->CmdBuffer.Size; ++cmd_index) {
      const ImDrawCmd& cmd = list->CmdBuffer[cmd_index];
      const auto tex_id = draw_cmd_texture_id(cmd);
      if (tex_id && texture_id_is_empty(*tex_id)) {
        return true;
      }
    }
  }
  return false;
}

#if IMGUI_VERSION_NUM >= 19000
template <typename T>
static auto* draw_data_textures(T* draw_data) {
  if constexpr (requires { draw_data->Textures; }) {
    if constexpr (std::is_pointer_v<decltype(draw_data->Textures)>) {
      return draw_data->Textures;
    } else {
      return &draw_data->Textures;
    }
  } else {
    return static_cast<void*>(nullptr);
  }
}

static ImTextureData* texture_ptr_from_entry(auto& entry) {
  if constexpr (std::is_pointer_v<std::decay_t<decltype(entry)>>) {
    return entry;
  } else {
    return &entry;
  }
}

static void process_texture_requests(ImDrawData* draw_data) {
  auto* textures = draw_data_textures(draw_data);
  if (!textures) {
    return;
  }
  for (int i = 0; i < textures->Size; ++i) {
    auto& entry = (*textures)[i];
    ImTextureData* tex = texture_ptr_from_entry(entry);
    if (!tex) {
      continue;
    }
    bool tex_empty = false;
    if constexpr (requires { tex->TexID; }) {
      tex_empty = texture_id_is_empty(tex->TexID);
    }
    if (tex_empty ||
        tex->Status == ImTextureStatus_WantCreate ||
        tex->Status == ImTextureStatus_WantUpdates ||
        tex->Status == ImTextureStatus_WantDestroy) {
      ImGui_ImplVulkan_UpdateTexture(tex);
    }
  }
}

static bool draw_data_has_pending_textures(const ImDrawData* draw_data) {
  auto* textures = draw_data_textures(const_cast<ImDrawData*>(draw_data));
  if (!textures) {
    return false;
  }
  for (int i = 0; i < textures->Size; ++i) {
    auto& entry = (*textures)[i];
    ImTextureData* tex = texture_ptr_from_entry(entry);
    if (!tex) {
      continue;
    }
    if (tex->Status == ImTextureStatus_WantCreate ||
        tex->Status == ImTextureStatus_WantUpdates ||
        tex->Status == ImTextureStatus_WantDestroy) {
      return true;
    }
    if constexpr (requires { tex->TexID; }) {
      if (texture_id_is_empty(tex->TexID)) {
        return true;
      }
    }
  }
  return false;
}
#endif

void maybe_refresh_ai_status() {
  const auto path = g_state.root / "build" / "ai_results.json";
  if (!std::filesystem::exists(path)) {
    g_state.ai_error = "no results";
    g_state.ai_valid = false;
    return;
  }
  std::error_code ec;
  auto ftime = std::filesystem::last_write_time(path, ec);
  if (ec) return;
  auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
      ftime - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());
  const uint64_t mtime = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::seconds>(sctp.time_since_epoch()).count());
  if (mtime == g_state.ai_mtime) {
    return;
  }
  g_state.ai_mtime = mtime;

  std::ifstream in(path);
  if (!in) return;
  std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (contents.empty()) {
    g_state.ai_error = "no results";
    g_state.ai_valid = false;
    return;
  }
  std::string error;
  if (rkg::parse_ai_results_summary(contents, g_state.ai_results, error)) {
    g_state.ai_valid = true;
    g_state.ai_error.clear();
  } else {
    g_state.ai_valid = false;
    g_state.ai_error = error.empty() ? "parse failed" : error;
    g_state.ai_goal.clear();
    g_state.ai_provider.clear();
    g_state.ai_model.clear();
  }

#if RKG_ENABLE_DATA_JSON
  if (g_state.ai_valid && !g_state.ai_results.run_id.empty()) {
    const auto run_info_path = g_state.root / "build" / "ai_runs" / g_state.ai_results.run_id / "run_info.json";
    std::error_code run_ec;
    auto run_time = std::filesystem::last_write_time(run_info_path, run_ec);
    if (!run_ec) {
      auto run_sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
          run_time - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());
      const uint64_t run_mtime = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::seconds>(run_sctp.time_since_epoch()).count());
      if (run_mtime != g_state.ai_info_mtime) {
        g_state.ai_info_mtime = run_mtime;
        std::ifstream run_in(run_info_path);
        if (run_in) {
          nlohmann::json run_info;
          run_in >> run_info;
          g_state.ai_goal = run_info.value("goal", "");
          g_state.ai_provider = run_info.value("provider", "");
          g_state.ai_model = run_info.value("model", "");
        }
      }
    }
  }
#endif
}

void maybe_refresh_viewport_texture() {
  const auto* viewport = rkg::get_vulkan_viewport();
  if (!viewport || !viewport->image_view || !viewport->sampler ||
      viewport->width == 0 || viewport->height == 0) {
    g_state.viewport_available = false;
    g_state.viewport_error = "viewport texture unavailable";
    return;
  }

  if (viewport->version != g_state.viewport_version || g_state.viewport_texture == nullptr) {
    g_state.viewport_version = viewport->version;
    g_state.viewport_width = viewport->width;
    g_state.viewport_height = viewport->height;
    g_state.viewport_texture = ImGui_ImplVulkan_AddTexture(
        reinterpret_cast<VkSampler>(viewport->sampler),
        reinterpret_cast<VkImageView>(viewport->image_view),
        static_cast<VkImageLayout>(viewport->layout));
  }
  g_state.viewport_available = g_state.viewport_texture != nullptr;
  g_state.viewport_error.clear();
}

} // namespace

bool init_vulkan() {
  if (g_state.initialized) return true;
  const auto* hooks = rkg::get_vulkan_hooks();
  if (!hooks) {
    rkg::log::warn("debug_ui: Vulkan hooks not available");
    return false;
  }
  g_state.instance = reinterpret_cast<VkInstance>(hooks->instance);
  g_state.physical = reinterpret_cast<VkPhysicalDevice>(hooks->physical_device);
  g_state.device = reinterpret_cast<VkDevice>(hooks->device);
  g_state.queue = reinterpret_cast<VkQueue>(hooks->queue);
  g_state.render_pass = reinterpret_cast<VkRenderPass>(hooks->render_pass);
  g_state.window = reinterpret_cast<SDL_Window*>(hooks->window);
  g_state.queue_family = hooks->queue_family;
  g_state.image_count = hooks->image_count;
  g_state.swapchain_format = hooks->swapchain_format != 0
                                 ? static_cast<VkFormat>(hooks->swapchain_format)
                                 : VK_FORMAT_UNDEFINED;
  rkg::log::info("debug_ui: swapchain_format=" + std::to_string(static_cast<int>(g_state.swapchain_format)));
  const bool has_render_pass = has_render_pass_member<ImGui_ImplVulkan_InitInfo>();
  const bool has_dynamic_rendering = has_dynamic_rendering_member<ImGui_ImplVulkan_InitInfo>();
  const bool has_color_format = has_color_format_member<ImGui_ImplVulkan_InitInfo>();
  const bool has_pipeline_rendering = has_pipeline_rendering_member<ImGui_ImplVulkan_InitInfo>();
  const bool can_render_pass = has_render_pass && g_state.render_pass != VK_NULL_HANDLE;
  const bool can_dynamic_rendering = has_dynamic_rendering && has_color_format && has_pipeline_rendering;
  g_state.render_inside_pass = can_render_pass;
  const bool use_dynamic_rendering = !g_state.render_inside_pass && can_dynamic_rendering;
  rkg::log::info(std::string("debug_ui: backend caps render_pass=") + (has_render_pass ? "yes" : "no") +
                 " dynamic=" + (has_dynamic_rendering ? "yes" : "no") +
                 " color_format=" + (has_color_format ? "yes" : "no") +
                 " pipeline_rendering=" + (has_pipeline_rendering ? "yes" : "no"));
  if (!can_render_pass && !can_dynamic_rendering) {
    rkg::log::warn("debug_ui: ImGui Vulkan backend lacks render pass and dynamic rendering support");
    return false;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  if (g_state.docking_enabled) {
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  }
  ImGui::StyleColorsDark();

  if (!ImGui_ImplSDL3_InitForVulkan(g_state.window)) {
    rkg::log::warn("debug_ui: ImGui SDL3 init failed");
    return false;
  }

  if (!create_descriptor_pool()) {
    rkg::log::warn("debug_ui: descriptor pool creation failed");
    return false;
  }

  ImGui_ImplVulkan_InitInfo init_info{};
  init_info.Instance = g_state.instance;
  init_info.PhysicalDevice = g_state.physical;
  init_info.Device = g_state.device;
  init_info.Queue = g_state.queue;
  init_info.QueueFamily = g_state.queue_family;
  init_info.DescriptorPool = g_state.descriptor_pool;
  init_info.MinImageCount = g_state.image_count;
  init_info.ImageCount = g_state.image_count;
  if (g_state.render_inside_pass) {
    set_render_pass(init_info, g_state.render_pass);
  } else {
    set_render_pass(init_info, VK_NULL_HANDLE);
  }
  set_dynamic_rendering(init_info, use_dynamic_rendering);
  if (use_dynamic_rendering) {
    if (g_state.swapchain_format == VK_FORMAT_UNDEFINED) {
      rkg::log::warn("debug_ui: dynamic rendering requires a valid swapchain format");
      return false;
    }
    set_color_format(init_info, g_state.swapchain_format);
    set_msaa_samples(init_info, VK_SAMPLE_COUNT_1_BIT);
    static VkPipelineRenderingCreateInfo rendering_info{};
    rendering_info.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
    rendering_info.pNext = nullptr;
    rendering_info.colorAttachmentCount = 1;
    rendering_info.pColorAttachmentFormats = &g_state.swapchain_format;
    set_pipeline_rendering_info(init_info, &rendering_info);
  }
  if (!ImGui_ImplVulkan_Init(&init_info)) {
    rkg::log::warn("debug_ui: ImGui Vulkan init failed");
    return false;
  }
#if IMGUI_VERSION_NUM >= 19000
  ImGui::GetIO().BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
#endif
  rkg::log::info(std::string("debug_ui: render path ") +
                 (g_state.render_inside_pass ? "render_pass" : "dynamic_rendering"));
  if (!g_state.render_inside_pass) {
    rkg::log::info("debug_ui: dynamic rendering scope owned by renderer");
  }

  g_state.initialized = true;
  g_state.has_draw_data = false;
  return true;
}

bool is_initialized() {
  return g_state.initialized;
}

void shutdown() {
  if (!g_state.initialized) return;
  vkDeviceWaitIdle(g_state.device);
  ImGui_ImplVulkan_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImGui::DestroyContext();
  if (g_state.descriptor_pool != VK_NULL_HANDLE) {
    vkDestroyDescriptorPool(g_state.device, g_state.descriptor_pool, nullptr);
    g_state.descriptor_pool = VK_NULL_HANDLE;
  }
  g_state.initialized = false;
  g_state.has_draw_data = false;
}

void new_frame(float dt_seconds) {
  if (!g_state.initialized || !g_state.visible) {
    return;
  }
  g_state.fps = dt_seconds > 0.0f ? (1.0f / dt_seconds) : 0.0f;
  ImGui_ImplVulkan_NewFrame();
  ImGui_ImplSDL3_NewFrame();
  ImGui::NewFrame();

  maybe_refresh_viewport_texture();

  if (g_state.show_builtin) {
    if (ImGui::Begin("Stats")) {
      ImGui::Text("FPS: %.1f", g_state.fps);
      ImGui::Text("Frame time: %.2f ms", dt_seconds * 1000.0f);
      ImGui::Text("Renderer: %s", g_state.renderer_name.c_str());
    }
    ImGui::End();

    if (ImGui::Begin("World")) {
      if (g_state.host && g_state.host->registry) {
        const auto count = g_state.host->registry->entity_count();
        ImGui::Text("Entities: %zu", count);
        const auto entities = g_state.host->registry->entities();
        if (!entities.empty()) {
          const auto entity = entities.front();
          if (auto* transform = g_state.host->registry->get_transform(entity)) {
            ImGui::Text("Entity %u", entity);
            ImGui::Text("Pos: %.2f %.2f %.2f",
                        transform->position[0],
                        transform->position[1],
                        transform->position[2]);
          }
        }
      } else {
        ImGui::Text("Entities: N/A");
      }
    }
    ImGui::End();

    if (ImGui::Begin("Hot Reload")) {
      ImGui::Text("Last reload: %s", g_state.last_reload_time.c_str());
      if (!g_state.last_reload_error.empty()) {
        ImGui::Text("Error: %s", g_state.last_reload_error.c_str());
      }
      if (ImGui::Button("Force Reload")) {
        g_state.force_reload = true;
      }
    }
    ImGui::End();

    if (ImGui::Begin("AI Orchestration")) {
      maybe_refresh_ai_status();
      if (!g_state.ai_valid) {
        ImGui::Text("Status: %s", g_state.ai_error.empty() ? "unknown" : g_state.ai_error.c_str());
      } else {
        ImGui::Text("Run: %s", g_state.ai_results.run_id.c_str());
        if (!g_state.ai_goal.empty()) {
          ImGui::TextWrapped("Goal: %s", g_state.ai_goal.c_str());
        }
        if (!g_state.ai_provider.empty() || !g_state.ai_model.empty()) {
          ImGui::Text("Provider: %s  Model: %s",
                      g_state.ai_provider.empty() ? "unknown" : g_state.ai_provider.c_str(),
                      g_state.ai_model.empty() ? "unknown" : g_state.ai_model.c_str());
        }
        ImGui::Text("Status: %s", g_state.ai_results.status.c_str());
        ImGui::Text("Mode: %s", g_state.ai_results.dry_run ? "dry-run" : "applied");
        ImGui::Text("Conflicts: %d", g_state.ai_results.conflicts);
        if (g_state.ai_results.has_context_drift) {
          ImGui::Text("Drift: %s", g_state.ai_results.drift_detected ? "mismatch" : "match");
          if (g_state.ai_results.drift_detected) {
            ImGui::Text("Changes: +%d -%d ~%d",
                        g_state.ai_results.drift_added,
                        g_state.ai_results.drift_removed,
                        g_state.ai_results.drift_modified);
          }
          if (!g_state.ai_results.drift_message.empty()) {
            ImGui::TextWrapped("Note: %s", g_state.ai_results.drift_message.c_str());
          }
        } else {
          ImGui::Text("Drift: N/A");
        }
        if (g_state.ai_results.status != "ok") {
          ImGui::TextWrapped("Last error: %s", g_state.ai_results.status.c_str());
        } else if (!g_state.ai_error.empty()) {
          ImGui::TextWrapped("Last error: %s", g_state.ai_error.c_str());
        } else {
          ImGui::Text("Last error: none");
        }
      }
    }
    ImGui::End();

    if (ImGui::Begin("Console")) {
      const auto lines = rkg::log::recent(200);
      for (const auto& line : lines) {
        ImGui::TextUnformatted(line.c_str());
      }
    }
    ImGui::End();
  }

  if (g_state.draw_callback) {
    g_state.draw_callback(g_state.draw_user);
  }

  ImGui::Render();
#if IMGUI_VERSION_NUM >= 19000
  process_texture_requests(ImGui::GetDrawData());
#endif
  g_state.has_draw_data = true;
}

void render(VkCommandBuffer cmd) {
  if (!g_state.initialized || !g_state.visible || !g_state.has_draw_data) {
    return;
  }
  ImDrawData* draw_data = ImGui::GetDrawData();
  if (!draw_data || !draw_data->Valid || draw_data->CmdListsCount == 0) {
    return;
  }
#if IMGUI_VERSION_NUM >= 19000
  process_texture_requests(draw_data);
  if (draw_data_has_pending_textures(draw_data)) {
    static bool warned_textures = false;
    if (!warned_textures) {
      warned_textures = true;
      rkg::log::warn("debug_ui: textures pending upload; skipping render this frame");
    }
    return;
  }
  {
    static bool logged_textures = false;
    if (!logged_textures) {
      auto* textures = draw_data_textures(draw_data);
      if (textures) {
        rkg::log::info("debug_ui: draw_data textures=" + std::to_string(textures->Size));
        const int count = textures->Size < 4 ? textures->Size : 4;
        for (int i = 0; i < count; ++i) {
          auto& entry = (*textures)[i];
          ImTextureData* tex = texture_ptr_from_entry(entry);
          if (!tex) {
            rkg::log::warn("debug_ui: texture entry " + std::to_string(i) + " null");
            continue;
          }
          bool tex_empty = false;
          if constexpr (requires { tex->TexID; }) {
            tex_empty = texture_id_is_empty(tex->TexID);
          }
          rkg::log::info("debug_ui: texture " + std::to_string(i) +
                         " status=" + std::to_string(static_cast<int>(tex->Status)) +
                         " tex_empty=" + std::string(tex_empty ? "true" : "false"));
        }
      }
      logged_textures = true;
    }
  }
#endif
  if (draw_data_has_empty_texture(draw_data)) {
    static bool warned = false;
    if (!warned) {
      warned = true;
      rkg::log::warn("debug_ui: draw data references empty texture; skipping render");
    }
    return;
  }
  {
    static bool logged = false;
    if (!logged) {
      const int cmd_lists = draw_data->CmdListsCount;
      const int total_vtx = draw_data->TotalVtxCount;
      const int total_idx = draw_data->TotalIdxCount;
      rkg::log::info("debug_ui: render draw_data lists=" + std::to_string(cmd_lists) +
                     " vtx=" + std::to_string(total_vtx) +
                     " idx=" + std::to_string(total_idx));
      logged = true;
    }
  }
  ImGui_ImplVulkan_RenderDrawData(draw_data, cmd);
}

bool render_inside_pass() {
  return g_state.render_inside_pass;
}

void set_visible(bool visible) {
  g_state.visible = visible;
}

bool is_visible() {
  return g_state.visible;
}

void process_event(const void* event) {
  if (!g_state.initialized || !event) {
    return;
  }
  const SDL_Event* sdl_event = static_cast<const SDL_Event*>(event);
  ImGui_ImplSDL3_ProcessEvent(sdl_event);
}

void set_host_context(void* host_context) {
  g_state.host = static_cast<rkg::HostContext*>(host_context);
}

void set_root_path(const std::filesystem::path& root) {
  g_state.root = root;
}

void set_renderer_name(const std::string& name) {
  g_state.renderer_name = name.empty() ? "Unknown" : name;
}

void set_hot_reload_status(const std::string& time, const std::string& error) {
  g_state.last_reload_time = time;
  g_state.last_reload_error = error;
}

bool consume_force_reload_request() {
  const bool requested = g_state.force_reload;
  g_state.force_reload = false;
  return requested;
}

void set_viewport_size(int width, int height) {
  if (width < 0 || height < 0) {
    return;
  }
  rkg::set_vulkan_viewport_request(static_cast<uint32_t>(width), static_cast<uint32_t>(height));
}

bool viewport_supported() {
  return g_state.viewport_available;
}

void* viewport_texture() {
  return g_state.viewport_texture;
}

void viewport_size(int* width, int* height) {
  if (width) *width = static_cast<int>(g_state.viewport_width);
  if (height) *height = static_cast<int>(g_state.viewport_height);
}

const char* viewport_error() {
  return g_state.viewport_error.empty() ? nullptr : g_state.viewport_error.c_str();
}

void set_draw_callback(DrawCallback callback, void* user_data) {
  g_state.draw_callback = callback;
  g_state.draw_user = user_data;
}

void set_show_builtin(bool show) {
  g_state.show_builtin = show;
}

void set_docking_enabled(bool enabled) {
  g_state.docking_enabled = enabled;
  if (g_state.initialized) {
    ImGuiIO& io = ImGui::GetIO();
    if (enabled) {
      io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    } else {
      io.ConfigFlags &= ~ImGuiConfigFlags_DockingEnable;
    }
  }
}

} // namespace rkg::debug_ui
