#include "rkg/host_context.h"
#include "rkg/log.h"
#include "rkg/plugin_api.h"
#include "rkg/renderer_hooks.h"
#include "rkg/paths.h"
#include "rkg/math.h"
#include "rkg_platform/platform.h"
#include "stb_image.h"

#if RKG_ENABLE_IMGUI
#include "rkg_debug_ui/imgui_api.h"
#endif
#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>
#include <vulkan/vulkan.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

namespace {
namespace fs = std::filesystem;

struct BonePose {
  int parent = -1;
  std::string name;
  float pos[3]{0.0f, 0.0f, 0.0f};
  float rot[3]{0.0f, 0.0f, 0.0f};
  float scale[3]{1.0f, 1.0f, 1.0f};
};

struct SkeletonAsset {
  std::vector<BonePose> bones;
  std::vector<rkg::Mat4> inverse_bind;
};

void compute_skin_matrices_from_bones(const std::vector<BonePose>& bones,
                                      const std::vector<rkg::Mat4>& inverse_bind,
                                      std::vector<rkg::Mat4>& out);

struct VulkanState {
  rkg::HostContext* ctx = nullptr;
  SDL_Window* window = nullptr;
  VkInstance instance = VK_NULL_HANDLE;
  VkSurfaceKHR surface = VK_NULL_HANDLE;
  VkPhysicalDevice physical_device = VK_NULL_HANDLE;
  VkDevice device = VK_NULL_HANDLE;
  VkQueue graphics_queue = VK_NULL_HANDLE;
  VkQueue present_queue = VK_NULL_HANDLE;
  uint32_t graphics_queue_family = 0;
  uint32_t present_queue_family = 0;
  VkSwapchainKHR swapchain = VK_NULL_HANDLE;
  VkFormat swapchain_format = VK_FORMAT_UNDEFINED;
  VkExtent2D swapchain_extent{};
  std::vector<VkImage> swapchain_images;
  std::vector<VkImageView> swapchain_image_views;
  std::vector<VkImageLayout> swapchain_image_layouts;
  VkRenderPass render_pass = VK_NULL_HANDLE;
  std::vector<VkFramebuffer> framebuffers;
  VkCommandPool command_pool = VK_NULL_HANDLE;
  std::vector<VkCommandBuffer> command_buffers;
  VkSemaphore image_available = VK_NULL_HANDLE;
  VkSemaphore render_finished = VK_NULL_HANDLE;
  VkFence in_flight_fence = VK_NULL_HANDLE;
  bool dynamic_rendering_enabled = false;
  PFN_vkCmdBeginRenderingKHR cmd_begin_rendering = nullptr;
  PFN_vkCmdEndRenderingKHR cmd_end_rendering = nullptr;
  bool swapchain_needs_rebuild = false;
  bool surface_lost = false;
  bool device_lost = false;
  bool skip_imgui_frame = false;

  VkImage offscreen_image = VK_NULL_HANDLE;
  VkDeviceMemory offscreen_memory = VK_NULL_HANDLE;
  VkImageView offscreen_view = VK_NULL_HANDLE;
  VkSampler offscreen_sampler = VK_NULL_HANDLE;
  VkRenderPass offscreen_render_pass = VK_NULL_HANDLE;
  VkFramebuffer offscreen_framebuffer = VK_NULL_HANDLE;
  VkExtent2D offscreen_extent{};
  uint64_t offscreen_version = 0;
  VkImageLayout offscreen_color_layout = VK_IMAGE_LAYOUT_UNDEFINED;

  VkImage offscreen_depth_image = VK_NULL_HANDLE;
  VkDeviceMemory offscreen_depth_memory = VK_NULL_HANDLE;
  VkImageView offscreen_depth_view = VK_NULL_HANDLE;
  VkFormat offscreen_depth_format = VK_FORMAT_D32_SFLOAT;
  VkImageLayout offscreen_depth_layout = VK_IMAGE_LAYOUT_UNDEFINED;

  VkPipeline viewport_pipeline = VK_NULL_HANDLE;
  VkPipeline viewport_line_pipeline = VK_NULL_HANDLE;
  VkPipelineLayout viewport_pipeline_layout = VK_NULL_HANDLE;
  VkBuffer viewport_cube_buffer = VK_NULL_HANDLE;
  VkDeviceMemory viewport_cube_memory = VK_NULL_HANDLE;
  VkBuffer viewport_quad_buffer = VK_NULL_HANDLE;
  VkDeviceMemory viewport_quad_memory = VK_NULL_HANDLE;
  VkBuffer viewport_line_buffer = VK_NULL_HANDLE;
  VkDeviceMemory viewport_line_memory = VK_NULL_HANDLE;
  uint32_t viewport_cube_vertex_count = 0;
  uint32_t viewport_quad_vertex_count = 0;
  uint32_t viewport_line_vertex_capacity = 0;

  // Phase 2B: textured static mesh demo resources.
  VkPipeline textured_pipeline = VK_NULL_HANDLE;
  VkPipeline textured_pipeline_translucent = VK_NULL_HANDLE;
  VkPipelineLayout textured_pipeline_layout = VK_NULL_HANDLE;
  VkDescriptorSetLayout textured_desc_layout = VK_NULL_HANDLE;
  VkDescriptorPool textured_desc_pool = VK_NULL_HANDLE;
  VkDescriptorSet textured_desc_set = VK_NULL_HANDLE;
  VkBuffer textured_vertex_buffer = VK_NULL_HANDLE;
  VkDeviceMemory textured_vertex_memory = VK_NULL_HANDLE;
  VkBuffer textured_index_buffer = VK_NULL_HANDLE;
  VkDeviceMemory textured_index_memory = VK_NULL_HANDLE;
  uint32_t textured_index_count = 0;
  bool textured_has_uv0 = false;
  float textured_base_color[4] = {0.65f, 0.65f, 0.65f, 1.0f};
  VkImage textured_image = VK_NULL_HANDLE;
  VkDeviceMemory textured_image_memory = VK_NULL_HANDLE;
  VkImageView textured_image_view = VK_NULL_HANDLE;
  VkSampler textured_sampler = VK_NULL_HANDLE;
  std::string textured_asset_name{};
  bool textured_ready = false;

  // Phase 4: skinned mesh demo resources.
  VkPipeline skinned_pipeline = VK_NULL_HANDLE;
  VkPipeline skinned_pipeline_translucent = VK_NULL_HANDLE;
  VkPipelineLayout skinned_pipeline_layout = VK_NULL_HANDLE;
  VkDescriptorSetLayout skinned_desc_layout = VK_NULL_HANDLE;
  VkDescriptorPool skinned_desc_pool = VK_NULL_HANDLE;
  VkDescriptorSet skinned_desc_set = VK_NULL_HANDLE;
  VkBuffer skinned_vertex_buffer = VK_NULL_HANDLE;
  VkDeviceMemory skinned_vertex_memory = VK_NULL_HANDLE;
  VkBuffer skinned_index_buffer = VK_NULL_HANDLE;
  VkDeviceMemory skinned_index_memory = VK_NULL_HANDLE;
  VkBuffer skinned_joint_buffer = VK_NULL_HANDLE;
  VkDeviceMemory skinned_joint_memory = VK_NULL_HANDLE;
  uint32_t skinned_index_count = 0;
  uint32_t skinned_joint_count = 0;
  bool skinned_ready = false;
  SkeletonAsset skinned_skeleton{};
  std::vector<rkg::Mat4> skinned_joint_mats{};
  bool skinned_test_walk = false;
  bool skinned_test_walk_logged = false;
  uint32_t skinned_test_left_thigh = UINT32_MAX;
  uint32_t skinned_test_left_calf = UINT32_MAX;
  uint32_t skinned_test_right_thigh = UINT32_MAX;
  uint32_t skinned_test_right_calf = UINT32_MAX;

  // Live/procedural rig drive state (Phase 4).
  bool skinned_live_enabled = false;
  bool skinned_live_logged = false;
  bool skinned_live_dump_logged = false;
  float skinned_live_phase = 0.0f;
  float skinned_live_fwd = 0.0f;
  float skinned_live_strafe = 0.0f;
  bool skinned_live_grounded = false;
  float frame_dt = 1.0f / 60.0f;

  // Cached bone indices for procedural rig drive.
  bool skinned_live_map_valid = false;
  uint32_t bone_hips = UINT32_MAX;
  uint32_t bone_spine = UINT32_MAX;
  uint32_t bone_chest = UINT32_MAX;
  uint32_t bone_neck = UINT32_MAX;
  uint32_t bone_head = UINT32_MAX;
  uint32_t bone_l_thigh = UINT32_MAX;
  uint32_t bone_l_calf = UINT32_MAX;
  uint32_t bone_l_foot = UINT32_MAX;
  uint32_t bone_r_thigh = UINT32_MAX;
  uint32_t bone_r_calf = UINT32_MAX;
  uint32_t bone_r_foot = UINT32_MAX;
  uint32_t bone_l_upper_arm = UINT32_MAX;
  uint32_t bone_l_lower_arm = UINT32_MAX;
  uint32_t bone_r_upper_arm = UINT32_MAX;
  uint32_t bone_r_lower_arm = UINT32_MAX;
};

VulkanState g_state{};

static bool g_log_steps = true;
static int g_log_step_index = 0;

static void log_step(const std::string& label) {
  if (!g_log_steps) {
    return;
  }
  rkg::log::info("renderer:vulkan step " + std::to_string(g_log_step_index++) + ": " + label);
}

static const char* vk_result_name(VkResult result) {
  switch (result) {
    case VK_SUCCESS:
      return "VK_SUCCESS";
    case VK_SUBOPTIMAL_KHR:
      return "VK_SUBOPTIMAL_KHR";
    case VK_ERROR_OUT_OF_DATE_KHR:
      return "VK_ERROR_OUT_OF_DATE_KHR";
    case VK_ERROR_SURFACE_LOST_KHR:
      return "VK_ERROR_SURFACE_LOST_KHR";
    case VK_ERROR_DEVICE_LOST:
      return "VK_ERROR_DEVICE_LOST";
    case VK_TIMEOUT:
      return "VK_TIMEOUT";
    case VK_NOT_READY:
      return "VK_NOT_READY";
#ifdef VK_ERROR_FULL_SCREEN_EXCLUSIVE_MODE_LOST_EXT
    case VK_ERROR_FULL_SCREEN_EXCLUSIVE_MODE_LOST_EXT:
      return "VK_ERROR_FULL_SCREEN_EXCLUSIVE_MODE_LOST_EXT";
#endif
    case VK_ERROR_OUT_OF_HOST_MEMORY:
      return "VK_ERROR_OUT_OF_HOST_MEMORY";
    case VK_ERROR_OUT_OF_DEVICE_MEMORY:
      return "VK_ERROR_OUT_OF_DEVICE_MEMORY";
    default:
      return "VK_ERROR_UNKNOWN";
  }
}

bool create_viewport_pipeline();
bool create_viewport_line_pipeline();
bool create_viewport_vertex_buffer();
bool create_textured_pipeline();
bool create_textured_pipeline_translucent();
bool create_skinned_pipeline();
bool create_skinned_pipeline_translucent();
bool load_textured_asset();
bool create_swapchain();
bool create_render_pass();
bool create_framebuffers();
bool create_command_buffers();
void destroy_swapchain(bool keep_render_pass = false);

static void get_window_drawable_size(int& w, int& h) {
  w = 0;
  h = 0;
#if SDL_VERSION_ATLEAST(3, 0, 0)
  SDL_GetWindowSizeInPixels(g_state.window, &w, &h);
#else
  SDL_GetWindowSize(g_state.window, &w, &h);
#endif
  if (w == 0 || h == 0) {
    SDL_GetWindowSize(g_state.window, &w, &h);
  }
}

static constexpr uint32_t kViewportVertexCount = 36;
static const float kViewportCubeVertices[kViewportVertexCount * 3] = {
    -0.5f, -0.5f, -0.5f,  0.5f, -0.5f, -0.5f,  0.5f,  0.5f, -0.5f,
     0.5f,  0.5f, -0.5f, -0.5f,  0.5f, -0.5f, -0.5f, -0.5f, -0.5f,

    -0.5f, -0.5f,  0.5f,  0.5f, -0.5f,  0.5f,  0.5f,  0.5f,  0.5f,
     0.5f,  0.5f,  0.5f, -0.5f,  0.5f,  0.5f, -0.5f, -0.5f,  0.5f,

    -0.5f,  0.5f,  0.5f, -0.5f,  0.5f, -0.5f, -0.5f, -0.5f, -0.5f,
    -0.5f, -0.5f, -0.5f, -0.5f, -0.5f,  0.5f, -0.5f,  0.5f,  0.5f,

     0.5f,  0.5f,  0.5f,  0.5f,  0.5f, -0.5f,  0.5f, -0.5f, -0.5f,
     0.5f, -0.5f, -0.5f,  0.5f, -0.5f,  0.5f,  0.5f,  0.5f,  0.5f,

    -0.5f, -0.5f, -0.5f,  0.5f, -0.5f, -0.5f,  0.5f, -0.5f,  0.5f,
     0.5f, -0.5f,  0.5f, -0.5f, -0.5f,  0.5f, -0.5f, -0.5f, -0.5f,

    -0.5f,  0.5f, -0.5f,  0.5f,  0.5f, -0.5f,  0.5f,  0.5f,  0.5f,
     0.5f,  0.5f,  0.5f, -0.5f,  0.5f,  0.5f, -0.5f,  0.5f, -0.5f,
};

static constexpr uint32_t kViewportQuadVertexCount = 6;
static const float kViewportQuadVertices[kViewportQuadVertexCount * 3] = {
    -0.5f, -0.5f, 0.0f,  0.5f, -0.5f, 0.0f,  0.5f,  0.5f, 0.0f,
     0.5f,  0.5f, 0.0f, -0.5f,  0.5f, 0.0f, -0.5f, -0.5f, 0.0f,
};

static const uint32_t rkg_viewport_vert_spv[] = {
  0x07230203u, 0x00010300u, 0x00070000u, 0x00000019u, 0x00000000u, 0x00020011u, 0x00000001u, 0x0003000eu,
  0x00000000u, 0x00000001u, 0x0007000fu, 0x00000000u, 0x00000001u, 0x6e69616du, 0x00000000u, 0x00000002u,
  0x00000003u, 0x00040005u, 0x00000001u, 0x6e69616du, 0x00000000u, 0x00040005u, 0x00000002u, 0x6f506e69u,
  0x00000073u, 0x00050005u, 0x00000003u, 0x505f6c67u, 0x7469736fu, 0x006e6f69u, 0x00060005u, 0x00000004u,
  0x68737550u, 0x736e6f43u, 0x746e6174u, 0x00000073u, 0x00040006u, 0x00000004u, 0x00000000u, 0x0070766du,
  0x00050006u, 0x00000004u, 0x00000001u, 0x6f6c6f63u, 0x00000072u, 0x00030005u, 0x00000005u, 0x00006370u,
  0x00040047u, 0x00000002u, 0x0000001eu, 0x00000000u, 0x00040047u, 0x00000003u, 0x0000000bu, 0x00000000u,
  0x00030047u, 0x00000004u, 0x00000002u, 0x00050048u, 0x00000004u, 0x00000000u, 0x00000023u, 0x00000000u,
  0x00040048u, 0x00000004u, 0x00000000u, 0x00000005u, 0x00050048u, 0x00000004u, 0x00000000u, 0x00000007u,
  0x00000010u, 0x00050048u, 0x00000004u, 0x00000001u, 0x00000023u, 0x00000040u, 0x00020013u, 0x00000006u,
  0x00030016u, 0x00000007u, 0x00000020u, 0x00040015u, 0x00000008u, 0x00000020u, 0x00000001u, 0x00040017u,
  0x00000009u, 0x00000007u, 0x00000003u, 0x00040017u, 0x0000000au, 0x00000007u, 0x00000004u, 0x00040018u,
  0x0000000bu, 0x0000000au, 0x00000004u, 0x0004001eu, 0x00000004u, 0x0000000bu, 0x0000000au, 0x00040020u,
  0x0000000cu, 0x00000009u, 0x00000004u, 0x00040020u, 0x0000000du, 0x00000009u, 0x0000000bu, 0x00040020u,
  0x0000000eu, 0x00000001u, 0x00000009u, 0x00040020u, 0x0000000fu, 0x00000003u, 0x0000000au, 0x00030021u,
  0x00000010u, 0x00000006u, 0x0004002bu, 0x00000007u, 0x00000011u, 0x3f800000u, 0x0004002bu, 0x00000008u,
  0x00000012u, 0x00000000u, 0x0004003bu, 0x0000000eu, 0x00000002u, 0x00000001u, 0x0004003bu, 0x0000000fu,
  0x00000003u, 0x00000003u, 0x0004003bu, 0x0000000cu, 0x00000005u, 0x00000009u, 0x00050036u, 0x00000006u,
  0x00000001u, 0x00000000u, 0x00000010u, 0x000200f8u, 0x00000013u, 0x0004003du, 0x00000009u, 0x00000014u,
  0x00000002u, 0x00050050u, 0x0000000au, 0x00000015u, 0x00000014u, 0x00000011u, 0x00050041u, 0x0000000du,
  0x00000016u, 0x00000005u, 0x00000012u, 0x0004003du, 0x0000000bu, 0x00000017u, 0x00000016u, 0x00050091u,
  0x0000000au, 0x00000018u, 0x00000017u, 0x00000015u, 0x0003003eu, 0x00000003u, 0x00000018u, 0x000100fdu,
  0x00010038u
};

static const uint32_t rkg_viewport_frag_spv[] = {
  0x07230203u, 0x00010300u, 0x00070000u, 0x00000012u, 0x00000000u, 0x00020011u, 0x00000001u, 0x0003000eu,
  0x00000000u, 0x00000001u, 0x0006000fu, 0x00000004u, 0x00000001u, 0x6e69616du, 0x00000000u, 0x00000002u,
  0x00030010u, 0x00000001u, 0x00000007u, 0x00040005u, 0x00000001u, 0x6e69616du, 0x00000000u, 0x00050005u,
  0x00000002u, 0x4374756fu, 0x726f6c6fu, 0x00000000u, 0x00060005u, 0x00000003u, 0x68737550u, 0x736e6f43u,
  0x746e6174u, 0x00000073u, 0x00040006u, 0x00000003u, 0x00000000u, 0x0070766du, 0x00050006u, 0x00000003u,
  0x00000001u, 0x6f6c6f63u, 0x00000072u, 0x00030005u, 0x00000004u, 0x00006370u, 0x00040047u, 0x00000002u,
  0x0000001eu, 0x00000000u, 0x00030047u, 0x00000003u, 0x00000002u, 0x00050048u, 0x00000003u, 0x00000000u,
  0x00000023u, 0x00000000u, 0x00040048u, 0x00000003u, 0x00000000u, 0x00000005u, 0x00050048u, 0x00000003u,
  0x00000000u, 0x00000007u, 0x00000010u, 0x00050048u, 0x00000003u, 0x00000001u, 0x00000023u, 0x00000040u,
  0x00020013u, 0x00000005u, 0x00030016u, 0x00000006u, 0x00000020u, 0x00040017u, 0x00000007u, 0x00000006u,
  0x00000004u, 0x00040018u, 0x00000008u, 0x00000007u, 0x00000004u, 0x0004001eu, 0x00000003u, 0x00000008u,
  0x00000007u, 0x00040020u, 0x00000009u, 0x00000009u, 0x00000003u, 0x00040020u, 0x0000000au, 0x00000009u,
  0x00000007u, 0x00040020u, 0x0000000bu, 0x00000003u, 0x00000007u, 0x00030021u, 0x0000000cu, 0x00000005u,
  0x00040015u, 0x0000000du, 0x00000020u, 0x00000001u, 0x0004002bu, 0x0000000du, 0x0000000eu, 0x00000001u,
  0x0004003bu, 0x0000000bu, 0x00000002u, 0x00000003u, 0x0004003bu, 0x00000009u, 0x00000004u, 0x00000009u,
  0x00050036u, 0x00000005u, 0x00000001u, 0x00000000u, 0x0000000cu, 0x000200f8u, 0x0000000fu, 0x00050041u,
  0x0000000au, 0x00000010u, 0x00000004u, 0x0000000eu, 0x0004003du, 0x00000007u, 0x00000011u, 0x00000010u,
  0x0003003eu, 0x00000002u, 0x00000011u, 0x000100fdu, 0x00010038u
};

bool create_instance() {
  Uint32 ext_count = 0;
  const char* const* ext_list = SDL_Vulkan_GetInstanceExtensions(&ext_count);
  if (!ext_list || ext_count == 0) {
    rkg::log::error("SDL_Vulkan_GetInstanceExtensions failed");
    return false;
  }
  std::vector<const char*> extensions(ext_list, ext_list + ext_count);

  VkApplicationInfo app_info{};
  app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app_info.pApplicationName = "rkg_minimal_app";
  app_info.apiVersion = VK_API_VERSION_1_1;

  VkInstanceCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  create_info.pApplicationInfo = &app_info;
  create_info.enabledExtensionCount = ext_count;
  create_info.ppEnabledExtensionNames = extensions.data();

  if (vkCreateInstance(&create_info, nullptr, &g_state.instance) != VK_SUCCESS) {
    rkg::log::error("vkCreateInstance failed");
    return false;
  }
  return true;
}

bool create_surface() {
  if (!SDL_Vulkan_CreateSurface(g_state.window, g_state.instance, nullptr, &g_state.surface)) {
    rkg::log::error("SDL_Vulkan_CreateSurface failed");
    return false;
  }
  return true;
}

bool pick_physical_device() {
  uint32_t device_count = 0;
  vkEnumeratePhysicalDevices(g_state.instance, &device_count, nullptr);
  if (device_count == 0) {
    rkg::log::error("No Vulkan physical devices found");
    return false;
  }
  std::vector<VkPhysicalDevice> devices(device_count);
  vkEnumeratePhysicalDevices(g_state.instance, &device_count, devices.data());
  g_state.physical_device = devices[0];
  return true;
}

bool find_queue_families() {
  uint32_t queue_family_count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(g_state.physical_device, &queue_family_count, nullptr);
  std::vector<VkQueueFamilyProperties> families(queue_family_count);
  vkGetPhysicalDeviceQueueFamilyProperties(g_state.physical_device, &queue_family_count, families.data());

  bool found_graphics = false;
  bool found_present = false;
  for (uint32_t i = 0; i < queue_family_count; ++i) {
    if (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
      g_state.graphics_queue_family = i;
      found_graphics = true;
    }
    VkBool32 present_supported = VK_FALSE;
    vkGetPhysicalDeviceSurfaceSupportKHR(g_state.physical_device, i, g_state.surface, &present_supported);
    if (present_supported) {
      g_state.present_queue_family = i;
      found_present = true;
    }
    if (found_graphics && found_present) {
      return true;
    }
  }
  return false;
}

bool create_device() {
  float queue_priority = 1.0f;
  std::vector<VkDeviceQueueCreateInfo> queue_infos;
  VkDeviceQueueCreateInfo graphics_queue{};
  graphics_queue.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  graphics_queue.queueFamilyIndex = g_state.graphics_queue_family;
  graphics_queue.queueCount = 1;
  graphics_queue.pQueuePriorities = &queue_priority;
  queue_infos.push_back(graphics_queue);

  if (g_state.present_queue_family != g_state.graphics_queue_family) {
    VkDeviceQueueCreateInfo present_queue{};
    present_queue.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    present_queue.queueFamilyIndex = g_state.present_queue_family;
    present_queue.queueCount = 1;
    present_queue.pQueuePriorities = &queue_priority;
    queue_infos.push_back(present_queue);
  }

  std::vector<const char*> extensions;
  extensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);

  g_state.dynamic_rendering_enabled = false;
  uint32_t ext_count = 0;
  if (vkEnumerateDeviceExtensionProperties(g_state.physical_device, nullptr, &ext_count, nullptr) == VK_SUCCESS &&
      ext_count > 0) {
    std::vector<VkExtensionProperties> ext_props(ext_count);
    if (vkEnumerateDeviceExtensionProperties(g_state.physical_device, nullptr, &ext_count, ext_props.data()) ==
        VK_SUCCESS) {
      for (const auto& ext : ext_props) {
        if (std::strcmp(ext.extensionName, VK_KHR_DYNAMIC_RENDERING_EXTENSION_NAME) == 0) {
          g_state.dynamic_rendering_enabled = true;
          break;
        }
      }
    }
  }
  if (g_state.dynamic_rendering_enabled) {
    extensions.push_back(VK_KHR_DYNAMIC_RENDERING_EXTENSION_NAME);
  }

  VkPhysicalDeviceDynamicRenderingFeaturesKHR dynamic_features{};
  if (g_state.dynamic_rendering_enabled) {
    dynamic_features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DYNAMIC_RENDERING_FEATURES_KHR;
    dynamic_features.dynamicRendering = VK_TRUE;
  }
  VkDeviceCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  create_info.queueCreateInfoCount = static_cast<uint32_t>(queue_infos.size());
  create_info.pQueueCreateInfos = queue_infos.data();
  create_info.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
  create_info.ppEnabledExtensionNames = extensions.data();
  if (g_state.dynamic_rendering_enabled) {
    create_info.pNext = &dynamic_features;
  }

  if (vkCreateDevice(g_state.physical_device, &create_info, nullptr, &g_state.device) != VK_SUCCESS) {
    rkg::log::error("vkCreateDevice failed");
    return false;
  }

  vkGetDeviceQueue(g_state.device, g_state.graphics_queue_family, 0, &g_state.graphics_queue);
  vkGetDeviceQueue(g_state.device, g_state.present_queue_family, 0, &g_state.present_queue);
  if (g_state.dynamic_rendering_enabled) {
    g_state.cmd_begin_rendering = reinterpret_cast<PFN_vkCmdBeginRenderingKHR>(
        vkGetDeviceProcAddr(g_state.device, "vkCmdBeginRenderingKHR"));
    g_state.cmd_end_rendering = reinterpret_cast<PFN_vkCmdEndRenderingKHR>(
        vkGetDeviceProcAddr(g_state.device, "vkCmdEndRenderingKHR"));
    if (!g_state.cmd_begin_rendering || !g_state.cmd_end_rendering) {
      rkg::log::warn("renderer:vulkan dynamic rendering entrypoints missing; disabling");
      g_state.dynamic_rendering_enabled = false;
      g_state.cmd_begin_rendering = nullptr;
      g_state.cmd_end_rendering = nullptr;
    }
  }
  return true;
}

uint32_t find_memory_type(uint32_t type_filter, VkMemoryPropertyFlags properties) {
  VkPhysicalDeviceMemoryProperties mem_properties;
  vkGetPhysicalDeviceMemoryProperties(g_state.physical_device, &mem_properties);
  for (uint32_t i = 0; i < mem_properties.memoryTypeCount; ++i) {
    if ((type_filter & (1 << i)) && (mem_properties.memoryTypes[i].propertyFlags & properties) == properties) {
      return i;
    }
  }
  return 0;
}

struct TexturedVertex {
  float pos[3];
  float uv[2];
};

struct SkinnedVertex {
  float pos[3];
  float uv[2];
  uint16_t joints[4];
  float weights[4];
};

bool read_file_bytes(const fs::path& path, std::vector<uint8_t>& out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  in.seekg(0, std::ios::end);
  const auto size = in.tellg();
  if (size <= 0) return false;
  out.resize(static_cast<size_t>(size));
  in.seekg(0, std::ios::beg);
  in.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(out.size()));
  return true;
}

bool update_host_buffer(const void* data, size_t byte_size, VkDeviceMemory memory) {
  if (!data || byte_size == 0 || memory == VK_NULL_HANDLE) return false;
  void* mapped = nullptr;
  if (vkMapMemory(g_state.device, memory, 0, byte_size, 0, &mapped) != VK_SUCCESS) {
    return false;
  }
  std::memcpy(mapped, data, byte_size);
  vkUnmapMemory(g_state.device, memory);
  return true;
}

bool create_host_buffer(const void* data,
                        size_t byte_size,
                        VkBufferUsageFlags usage,
                        VkBuffer& buffer,
                        VkDeviceMemory& memory) {
  VkBufferCreateInfo buffer_info{};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = static_cast<VkDeviceSize>(byte_size);
  buffer_info.usage = usage;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  if (vkCreateBuffer(g_state.device, &buffer_info, nullptr, &buffer) != VK_SUCCESS) {
    return false;
  }
  VkMemoryRequirements mem_req{};
  vkGetBufferMemoryRequirements(g_state.device, buffer, &mem_req);
  VkMemoryAllocateInfo alloc_info{};
  alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  alloc_info.allocationSize = mem_req.size;
  alloc_info.memoryTypeIndex =
      find_memory_type(mem_req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
  if (vkAllocateMemory(g_state.device, &alloc_info, nullptr, &memory) != VK_SUCCESS) {
    vkDestroyBuffer(g_state.device, buffer, nullptr);
    buffer = VK_NULL_HANDLE;
    return false;
  }
  vkBindBufferMemory(g_state.device, buffer, memory, 0);
  if (data && byte_size > 0) {
    void* mapped = nullptr;
    if (vkMapMemory(g_state.device, memory, 0, byte_size, 0, &mapped) == VK_SUCCESS) {
      std::memcpy(mapped, data, byte_size);
      vkUnmapMemory(g_state.device, memory);
    }
  }
  return true;
}

VkCommandBuffer begin_one_time_commands() {
  VkCommandBufferAllocateInfo alloc_info{};
  alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  alloc_info.commandPool = g_state.command_pool;
  alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  alloc_info.commandBufferCount = 1;
  VkCommandBuffer cmd = VK_NULL_HANDLE;
  if (vkAllocateCommandBuffers(g_state.device, &alloc_info, &cmd) != VK_SUCCESS) {
    return VK_NULL_HANDLE;
  }
  VkCommandBufferBeginInfo begin_info{};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  vkBeginCommandBuffer(cmd, &begin_info);
  return cmd;
}

void end_one_time_commands(VkCommandBuffer cmd) {
  if (cmd == VK_NULL_HANDLE) return;
  vkEndCommandBuffer(cmd);
  VkSubmitInfo submit{};
  submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit.commandBufferCount = 1;
  submit.pCommandBuffers = &cmd;
  vkQueueSubmit(g_state.graphics_queue, 1, &submit, VK_NULL_HANDLE);
  vkQueueWaitIdle(g_state.graphics_queue);
  vkFreeCommandBuffers(g_state.device, g_state.command_pool, 1, &cmd);
}

bool create_image(uint32_t width,
                  uint32_t height,
                  VkFormat format,
                  VkImageUsageFlags usage,
                  VkImage& image,
                  VkDeviceMemory& memory) {
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.format = format;
  image_info.extent = {width, height, 1};
  image_info.mipLevels = 1;
  image_info.arrayLayers = 1;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.usage = usage;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  if (vkCreateImage(g_state.device, &image_info, nullptr, &image) != VK_SUCCESS) {
    return false;
  }
  VkMemoryRequirements mem_req{};
  vkGetImageMemoryRequirements(g_state.device, image, &mem_req);
  VkMemoryAllocateInfo alloc_info{};
  alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  alloc_info.allocationSize = mem_req.size;
  alloc_info.memoryTypeIndex = find_memory_type(mem_req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  if (vkAllocateMemory(g_state.device, &alloc_info, nullptr, &memory) != VK_SUCCESS) {
    vkDestroyImage(g_state.device, image, nullptr);
    image = VK_NULL_HANDLE;
    return false;
  }
  vkBindImageMemory(g_state.device, image, memory, 0);
  return true;
}

bool transition_image_layout(VkImage image, VkImageLayout old_layout, VkImageLayout new_layout) {
  VkCommandBuffer cmd = begin_one_time_commands();
  if (cmd == VK_NULL_HANDLE) return false;
  VkImageMemoryBarrier barrier{};
  barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
  barrier.oldLayout = old_layout;
  barrier.newLayout = new_layout;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.image = image;
  barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  barrier.subresourceRange.baseMipLevel = 0;
  barrier.subresourceRange.levelCount = 1;
  barrier.subresourceRange.baseArrayLayer = 0;
  barrier.subresourceRange.layerCount = 1;
  VkPipelineStageFlags src_stage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
  VkPipelineStageFlags dst_stage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
  if (old_layout == VK_IMAGE_LAYOUT_UNDEFINED && new_layout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
    barrier.srcAccessMask = 0;
    barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    src_stage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
    dst_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
  } else if (old_layout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL &&
             new_layout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    src_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    dst_stage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
  }
  vkCmdPipelineBarrier(cmd, src_stage, dst_stage, 0, 0, nullptr, 0, nullptr, 1, &barrier);
  end_one_time_commands(cmd);
  return true;
}

bool copy_buffer_to_image(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height) {
  VkCommandBuffer cmd = begin_one_time_commands();
  if (cmd == VK_NULL_HANDLE) return false;
  VkBufferImageCopy region{};
  region.bufferOffset = 0;
  region.bufferRowLength = 0;
  region.bufferImageHeight = 0;
  region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  region.imageSubresource.mipLevel = 0;
  region.imageSubresource.baseArrayLayer = 0;
  region.imageSubresource.layerCount = 1;
  region.imageOffset = {0, 0, 0};
  region.imageExtent = {width, height, 1};
  vkCmdCopyBufferToImage(cmd, buffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
  end_one_time_commands(cmd);
  return true;
}

bool create_texture_from_rgba(const uint8_t* rgba, uint32_t width, uint32_t height) {
  if (!rgba || width == 0 || height == 0) return false;
  const size_t byte_count = static_cast<size_t>(width) * static_cast<size_t>(height) * 4;
  VkBuffer staging = VK_NULL_HANDLE;
  VkDeviceMemory staging_mem = VK_NULL_HANDLE;
  if (!create_host_buffer(rgba, byte_count, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, staging, staging_mem)) {
    return false;
  }
  if (!create_image(width, height, VK_FORMAT_R8G8B8A8_SRGB,
                    VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                    g_state.textured_image, g_state.textured_image_memory)) {
    vkDestroyBuffer(g_state.device, staging, nullptr);
    vkFreeMemory(g_state.device, staging_mem, nullptr);
    return false;
  }
  transition_image_layout(g_state.textured_image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
  copy_buffer_to_image(staging, g_state.textured_image, width, height);
  transition_image_layout(g_state.textured_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  vkDestroyBuffer(g_state.device, staging, nullptr);
  vkFreeMemory(g_state.device, staging_mem, nullptr);

  VkImageViewCreateInfo view_info{};
  view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_info.image = g_state.textured_image;
  view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  view_info.format = VK_FORMAT_R8G8B8A8_SRGB;
  view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  view_info.subresourceRange.baseMipLevel = 0;
  view_info.subresourceRange.levelCount = 1;
  view_info.subresourceRange.baseArrayLayer = 0;
  view_info.subresourceRange.layerCount = 1;
  if (vkCreateImageView(g_state.device, &view_info, nullptr, &g_state.textured_image_view) != VK_SUCCESS) {
    return false;
  }

  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.maxAnisotropy = 1.0f;
  if (vkCreateSampler(g_state.device, &sampler_info, nullptr, &g_state.textured_sampler) != VK_SUCCESS) {
    return false;
  }
  return true;
}

fs::path select_asset_dir(std::string& asset_name) {
  auto paths = rkg::resolve_paths(nullptr, std::nullopt, "demo_game");
  fs::path assets_dir = paths.content_root / "assets";
  fs::path repo_root = paths.content_root.parent_path().parent_path().parent_path();
  fs::path generated_assets_dir = repo_root / "build" / "content_cache" / "generated_assets";
  const bool assets_ok = fs::exists(assets_dir) && fs::is_directory(assets_dir);
  const bool generated_ok = fs::exists(generated_assets_dir) && fs::is_directory(generated_assets_dir);
  if (!assets_ok && !generated_ok) {
    return {};
  }
  auto resolve_in_dir = [&](const fs::path& root, const std::string& name) -> fs::path {
    if (root.empty()) return {};
    fs::path candidate = root / name;
    if (fs::exists(candidate / "asset.json")) return candidate;
    return {};
  };
  if (const char* env = std::getenv("RKG_RENDER_ASSET")) {
    if (env[0] != '\0') {
      if (auto candidate = resolve_in_dir(assets_dir, env); !candidate.empty()) {
        asset_name = env;
        return candidate;
      }
      if (auto candidate = resolve_in_dir(generated_assets_dir, env); !candidate.empty()) {
        asset_name = env;
        return candidate;
      }
    }
  }
  if (auto manny = resolve_in_dir(generated_assets_dir, "manny"); !manny.empty()) {
    asset_name = "manny";
    return manny;
  }
  if (auto manny = resolve_in_dir(assets_dir, "manny"); !manny.empty()) {
    asset_name = "manny";
    return manny;
  }
  if (auto testmanny = resolve_in_dir(generated_assets_dir, "testmanny"); !testmanny.empty()) {
    asset_name = "testmanny";
    return testmanny;
  }
  if (auto testmanny = resolve_in_dir(assets_dir, "testmanny"); !testmanny.empty()) {
    asset_name = "testmanny";
    return testmanny;
  }
  std::vector<fs::path> dirs;
  if (assets_ok) {
    for (const auto& entry : fs::directory_iterator(assets_dir)) {
      if (entry.is_directory() && fs::exists(entry.path() / "asset.json")) {
        dirs.push_back(entry.path());
      }
    }
  }
  if (generated_ok) {
    for (const auto& entry : fs::directory_iterator(generated_assets_dir)) {
      if (entry.is_directory() && fs::exists(entry.path() / "asset.json")) {
        dirs.push_back(entry.path());
      }
    }
  }
  if (dirs.empty()) return {};
  std::sort(dirs.begin(), dirs.end(),
            [](const fs::path& a, const fs::path& b) { return a.filename().string() < b.filename().string(); });
  const bool prefer_textured = std::getenv("RKG_RENDER_TEXTURED") != nullptr;
  if (prefer_textured) {
    for (const auto& dir : dirs) {
      const fs::path textures = dir / "textures";
      if (fs::exists(textures) && fs::is_directory(textures)) {
        for (const auto& tex : fs::directory_iterator(textures)) {
          if (tex.is_regular_file()) {
            asset_name = dir.filename().string();
            return dir;
          }
        }
      }
    }
  }
  asset_name = dirs.front().filename().string();
  return dirs.front();
}

struct MeshBinRaw {
  uint32_t vertex_count = 0;
  uint32_t index_count = 0;
  bool has_normals = false;
  bool has_uv0 = false;
  bool has_tangents = false;
  bool has_joints = false;
  bool has_weights = false;
  std::vector<float> positions;
  std::vector<float> uvs;
  std::vector<uint16_t> joints;
  std::vector<float> weights;
  std::vector<uint32_t> indices;
};

bool load_mesh_bin_raw(const fs::path& path, MeshBinRaw& out) {
  std::vector<uint8_t> bytes;
  if (!read_file_bytes(path, bytes)) return false;
  if (bytes.size() < sizeof(uint32_t) * 5) return false;
  const uint32_t* header = reinterpret_cast<const uint32_t*>(bytes.data());
  const uint32_t magic = header[0];
  const uint32_t version = header[1];
  if (magic != 0x30474B52 || (version != 1 && version != 2)) return false;
  out.vertex_count = header[2];
  out.index_count = header[3];
  const uint32_t flags = header[4];
  out.has_normals = (flags & 1u) != 0;
  out.has_uv0 = (flags & 2u) != 0;
  out.has_tangents = (flags & 4u) != 0;
  out.has_joints = (flags & 8u) != 0;
  out.has_weights = (flags & 16u) != 0;

  size_t offset = sizeof(uint32_t) * 5;
  const size_t pos_bytes = static_cast<size_t>(out.vertex_count) * 3 * sizeof(float);
  if (bytes.size() < offset + pos_bytes) return false;
  out.positions.resize(out.vertex_count * 3);
  std::memcpy(out.positions.data(), bytes.data() + offset, pos_bytes);
  offset += pos_bytes;

  if (out.has_normals) {
    const size_t norm_bytes = static_cast<size_t>(out.vertex_count) * 3 * sizeof(float);
    if (bytes.size() < offset + norm_bytes) return false;
    offset += norm_bytes;
  }
  if (out.has_uv0) {
    const size_t uv_bytes = static_cast<size_t>(out.vertex_count) * 2 * sizeof(float);
    if (bytes.size() < offset + uv_bytes) return false;
    out.uvs.resize(out.vertex_count * 2);
    std::memcpy(out.uvs.data(), bytes.data() + offset, uv_bytes);
    offset += uv_bytes;
  }
  if (out.has_tangents) {
    const size_t tan_bytes = static_cast<size_t>(out.vertex_count) * 4 * sizeof(float);
    if (bytes.size() < offset + tan_bytes) return false;
    offset += tan_bytes;
  }
  if (out.has_joints) {
    const size_t joint_bytes = static_cast<size_t>(out.vertex_count) * 4 * sizeof(uint16_t);
    if (bytes.size() < offset + joint_bytes) return false;
    out.joints.resize(out.vertex_count * 4);
    std::memcpy(out.joints.data(), bytes.data() + offset, joint_bytes);
    offset += joint_bytes;
  }
  if (out.has_weights) {
    const size_t weight_bytes = static_cast<size_t>(out.vertex_count) * 4 * sizeof(float);
    if (bytes.size() < offset + weight_bytes) return false;
    out.weights.resize(out.vertex_count * 4);
    std::memcpy(out.weights.data(), bytes.data() + offset, weight_bytes);
    offset += weight_bytes;
  }

  const size_t idx_bytes = static_cast<size_t>(out.index_count) * sizeof(uint32_t);
  if (bytes.size() < offset + idx_bytes) return false;
  out.indices.resize(out.index_count);
  std::memcpy(out.indices.data(), bytes.data() + offset, idx_bytes);
  return true;
}

bool load_mesh_bin_textured(const fs::path& path,
                            std::vector<TexturedVertex>& vertices,
                            std::vector<uint32_t>& indices,
                            bool& has_uv) {
  MeshBinRaw raw;
  if (!load_mesh_bin_raw(path, raw)) return false;
  has_uv = raw.has_uv0;
  vertices.resize(raw.vertex_count);
  for (uint32_t i = 0; i < raw.vertex_count; ++i) {
    vertices[i].pos[0] = raw.positions[i * 3 + 0];
    vertices[i].pos[1] = raw.positions[i * 3 + 1];
    vertices[i].pos[2] = raw.positions[i * 3 + 2];
    if (raw.has_uv0 && raw.uvs.size() >= (i * 2 + 2)) {
      vertices[i].uv[0] = raw.uvs[i * 2 + 0];
      vertices[i].uv[1] = raw.uvs[i * 2 + 1];
    } else {
      vertices[i].uv[0] = 0.0f;
      vertices[i].uv[1] = 0.0f;
    }
  }
  indices = std::move(raw.indices);
  return true;
}

bool load_mesh_bin_skinned(const fs::path& path,
                           std::vector<SkinnedVertex>& vertices,
                           std::vector<uint32_t>& indices,
                           bool& has_uv) {
  MeshBinRaw raw;
  if (!load_mesh_bin_raw(path, raw)) return false;
  if (!raw.has_joints || !raw.has_weights) return false;
  has_uv = raw.has_uv0;
  vertices.resize(raw.vertex_count);
  for (uint32_t i = 0; i < raw.vertex_count; ++i) {
    vertices[i].pos[0] = raw.positions[i * 3 + 0];
    vertices[i].pos[1] = raw.positions[i * 3 + 1];
    vertices[i].pos[2] = raw.positions[i * 3 + 2];
    if (raw.has_uv0 && raw.uvs.size() >= (i * 2 + 2)) {
      vertices[i].uv[0] = raw.uvs[i * 2 + 0];
      vertices[i].uv[1] = raw.uvs[i * 2 + 1];
    } else {
      vertices[i].uv[0] = 0.0f;
      vertices[i].uv[1] = 0.0f;
    }
    vertices[i].joints[0] = raw.joints[i * 4 + 0];
    vertices[i].joints[1] = raw.joints[i * 4 + 1];
    vertices[i].joints[2] = raw.joints[i * 4 + 2];
    vertices[i].joints[3] = raw.joints[i * 4 + 3];
    vertices[i].weights[0] = raw.weights[i * 4 + 0];
    vertices[i].weights[1] = raw.weights[i * 4 + 1];
    vertices[i].weights[2] = raw.weights[i * 4 + 2];
    vertices[i].weights[3] = raw.weights[i * 4 + 3];
  }
  indices = std::move(raw.indices);
  return true;
}

static bool env_flag_enabled(const char* name) {
  const char* val = std::getenv(name);
  if (!val) return false;
  if (std::strcmp(val, "0") == 0) return false;
  if (std::strcmp(val, "false") == 0) return false;
  if (std::strcmp(val, "FALSE") == 0) return false;
  return true;
}

static std::string to_lower(std::string s) {
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

static bool name_has(const std::string& name, const std::vector<std::string>& needles) {
  for (const auto& needle : needles) {
    if (name.find(needle) != std::string::npos) return true;
  }
  return false;
}

static uint32_t find_bone_index_by_name(const SkeletonAsset& skel,
                                        const std::vector<std::string>& include,
                                        const std::vector<std::string>& exclude) {
  for (uint32_t i = 0; i < skel.bones.size(); ++i) {
    const std::string n = to_lower(skel.bones[i].name);
    if (n.empty()) continue;
    if (!include.empty() && !name_has(n, include)) continue;
    if (!exclude.empty() && name_has(n, exclude)) continue;
    return i;
  }
  return UINT32_MAX;
}

static uint32_t find_side_bone(const SkeletonAsset& skel,
                               const std::vector<std::string>& side,
                               const std::vector<std::string>& part,
                               const std::vector<std::string>& exclude = {}) {
  for (uint32_t i = 0; i < skel.bones.size(); ++i) {
    const std::string n = to_lower(skel.bones[i].name);
    if (n.empty()) continue;
    if (!name_has(n, side)) continue;
    if (!name_has(n, part)) continue;
    if (!exclude.empty() && name_has(n, exclude)) continue;
    return i;
  }
  return UINT32_MAX;
}

static void build_live_bone_map() {
  const auto& skel = g_state.skinned_skeleton;
  if (skel.bones.empty()) return;

  const std::vector<std::string> left_tags = {"left", "_l", ".l", " l", "l_", "l-", " left"};
  const std::vector<std::string> right_tags = {"right", "_r", ".r", " r", "r_", "r-", " right"};
  const std::vector<std::string> hips_tags = {"hips", "pelvis", "hip", "root", "body", "center"};
  const std::vector<std::string> spine_tags = {"spine"};
  const std::vector<std::string> neck_tags = {"neck", "neck_01"};
  const std::vector<std::string> head_tags = {"head"};
  const std::vector<std::string> foot_tags = {"foot", "ankle", "ball", "toe"};

  g_state.bone_hips = find_bone_index_by_name(skel, hips_tags, {});
  g_state.bone_spine = find_bone_index_by_name(skel, spine_tags, {"spine02", "spine01"});
  g_state.bone_chest = find_bone_index_by_name(skel, {"spine02", "spine_02", "spine2"}, {});
  if (g_state.bone_chest == UINT32_MAX) {
    g_state.bone_chest = find_bone_index_by_name(skel, {"spine01", "spine_01", "spine1"}, {});
  }
  g_state.bone_neck = find_bone_index_by_name(skel, neck_tags, {});
  g_state.bone_head = find_bone_index_by_name(skel, head_tags, {});

  g_state.bone_l_thigh = find_side_bone(skel, left_tags, {"upleg", "upperleg", "thigh"});
  g_state.bone_r_thigh = find_side_bone(skel, right_tags, {"upleg", "upperleg", "thigh"});
  g_state.bone_l_calf = find_side_bone(skel, left_tags, {"leg", "calf", "lowerleg", "shin"}, {"upleg", "upperleg", "thigh"});
  g_state.bone_r_calf = find_side_bone(skel, right_tags, {"leg", "calf", "lowerleg", "shin"}, {"upleg", "upperleg", "thigh"});
  g_state.bone_l_foot = find_side_bone(skel, left_tags, foot_tags);
  g_state.bone_r_foot = find_side_bone(skel, right_tags, foot_tags);
  g_state.bone_l_upper_arm = find_side_bone(skel, left_tags, {"arm", "upperarm", "uparm"}, {"forearm", "lowerarm", "shoulder"});
  g_state.bone_r_upper_arm = find_side_bone(skel, right_tags, {"arm", "upperarm", "uparm"}, {"forearm", "lowerarm", "shoulder"});
  g_state.bone_l_lower_arm = find_side_bone(skel, left_tags, {"forearm", "lowerarm"});
  g_state.bone_r_lower_arm = find_side_bone(skel, right_tags, {"forearm", "lowerarm"});

  if (g_state.bone_l_thigh == UINT32_MAX || g_state.bone_r_thigh == UINT32_MAX) {
    const uint32_t any_thigh = find_bone_index_by_name(skel, {"upleg", "upperleg", "thigh"}, {});
    if (g_state.bone_l_thigh == UINT32_MAX) g_state.bone_l_thigh = any_thigh;
    if (g_state.bone_r_thigh == UINT32_MAX) g_state.bone_r_thigh = any_thigh;
  }
  if (g_state.bone_l_calf == UINT32_MAX || g_state.bone_r_calf == UINT32_MAX) {
    const uint32_t any_calf = find_bone_index_by_name(skel, {"leg", "calf", "lowerleg"}, {"upleg", "upperleg", "thigh"});
    if (g_state.bone_l_calf == UINT32_MAX) g_state.bone_l_calf = any_calf;
    if (g_state.bone_r_calf == UINT32_MAX) g_state.bone_r_calf = any_calf;
  }

  g_state.skinned_live_map_valid = true;
}

static void apply_live_pose(const SkeletonAsset& skel,
                            float phase,
                            float speed_norm,
                            float fwd,
                            float strafe,
                            bool grounded,
                            std::vector<BonePose>& out) {
  out = skel.bones;

  auto add_rot = [&](uint32_t idx, float rx, float ry, float rz) {
    if (idx == UINT32_MAX || idx >= out.size()) return;
    out[idx].rot[0] += rx;
    out[idx].rot[1] += ry;
    out[idx].rot[2] += rz;
  };
  auto add_pos = [&](uint32_t idx, float px, float py, float pz) {
    if (idx == UINT32_MAX || idx >= out.size()) return;
    out[idx].pos[0] += px;
    out[idx].pos[1] += py;
    out[idx].pos[2] += pz;
  };

  const float stride = std::min(std::max(speed_norm, 0.0f), 1.0f);
  const float stride_ease = stride * stride * (3.0f - 2.0f * stride);
  const float swing = std::sin(phase);
  const float swing2 = std::sin(phase * 2.0f);
  const float grounded_scale = grounded ? 1.0f : 0.45f;

  const float pelvis_bob = 0.035f * stride_ease * grounded_scale;
  const float pelvis_sway = 0.04f * stride_ease * grounded_scale;
  add_pos(g_state.bone_hips, pelvis_sway * swing, pelvis_bob * swing2, 0.0f);
  add_rot(g_state.bone_hips, -fwd * 0.18f * stride_ease, 0.0f, -strafe * 0.22f * stride_ease);

  add_rot(g_state.bone_spine, fwd * 0.16f * stride_ease, 0.0f, strafe * 0.18f * stride_ease);
  add_rot(g_state.bone_chest, fwd * 0.12f * stride_ease, 0.0f, strafe * 0.12f * stride_ease);
  add_rot(g_state.bone_neck, fwd * 0.06f * stride_ease, 0.0f, strafe * 0.06f * stride_ease);
  add_rot(g_state.bone_head, fwd * 0.04f * stride_ease, 0.0f, strafe * 0.05f * stride_ease);

  const float thigh_amp = 0.85f * stride_ease * grounded_scale + 0.05f;
  const float calf_amp = 0.65f * stride_ease * grounded_scale;
  const float foot_amp = 0.22f * stride_ease * grounded_scale;
  add_rot(g_state.bone_l_thigh, thigh_amp * swing, 0.0f, strafe * 0.12f * stride_ease);
  add_rot(g_state.bone_r_thigh, -thigh_amp * swing, 0.0f, -strafe * 0.12f * stride_ease);
  add_rot(g_state.bone_l_calf, calf_amp * std::max(0.0f, -swing), 0.0f, 0.0f);
  add_rot(g_state.bone_r_calf, calf_amp * std::max(0.0f, swing), 0.0f, 0.0f);
  add_rot(g_state.bone_l_foot, -foot_amp * std::max(0.0f, swing), 0.0f, 0.0f);
  add_rot(g_state.bone_r_foot, -foot_amp * std::max(0.0f, -swing), 0.0f, 0.0f);

  const float arm_amp = 0.6f * stride_ease * grounded_scale + 0.06f;
  add_rot(g_state.bone_l_upper_arm, -arm_amp * swing, 0.0f, 0.0f);
  add_rot(g_state.bone_r_upper_arm, arm_amp * swing, 0.0f, 0.0f);
  add_rot(g_state.bone_l_lower_arm, -0.25f * arm_amp * swing, 0.0f, 0.0f);
  add_rot(g_state.bone_r_lower_arm, 0.25f * arm_amp * swing, 0.0f, 0.0f);
}

static void update_skinned_live_pose() {
  g_state.skinned_live_enabled = env_flag_enabled("RKG_SKIN_LIVE") ||
                                 env_flag_enabled("RKG_SKIN_TEST_WALK") ||
                                 rkg::get_vulkan_viewport_skinned_live_enabled() ||
                                 rkg::get_vulkan_viewport_skinned_test_walk_enabled();
  if (!g_state.skinned_live_enabled || !g_state.skinned_ready) return;
  if (g_state.skinned_skeleton.bones.empty() || g_state.skinned_joint_count == 0) return;
  if (g_state.skinned_joint_memory == VK_NULL_HANDLE) return;

  float fwd = 0.0f;
  float strafe = 0.0f;
  bool grounded = false;
  rkg::get_vulkan_viewport_skinned_live_params(fwd, strafe, grounded);
  const float speed = std::sqrt(fwd * fwd + strafe * strafe);
  const float speed_norm = std::min(speed / 4.0f, 1.0f);
  const float speed_ease = speed_norm * speed_norm * (3.0f - 2.0f * speed_norm);

  if (!g_state.skinned_live_map_valid) {
    build_live_bone_map();
  }

  const float dt = std::max(1.0f / 240.0f, std::min(g_state.frame_dt, 1.0f / 12.0f));
  const float cadence = 0.02f + speed_ease * 0.06f;
  g_state.skinned_live_phase += dt * cadence * 6.2831853f;
  if (g_state.skinned_live_phase > 6.2831853f) {
    g_state.skinned_live_phase -= 6.2831853f;
  }

  std::vector<BonePose> posed;
  apply_live_pose(g_state.skinned_skeleton, g_state.skinned_live_phase, speed_ease,
                  fwd, strafe, grounded, posed);

  if (!g_state.skinned_live_logged) {
    auto name_or_idx = [&](uint32_t idx) -> std::string {
      if (idx == UINT32_MAX) return "none";
      if (idx < g_state.skinned_skeleton.bones.size() &&
          !g_state.skinned_skeleton.bones[idx].name.empty()) {
        return g_state.skinned_skeleton.bones[idx].name;
      }
      return std::string("#") + std::to_string(idx);
    };
    rkg::log::info("renderer:vulkan skinned live rig map: hips=" + name_or_idx(g_state.bone_hips) +
                   " spine=" + name_or_idx(g_state.bone_spine) +
                   " chest=" + name_or_idx(g_state.bone_chest) +
                   " neck=" + name_or_idx(g_state.bone_neck) +
                   " head=" + name_or_idx(g_state.bone_head) +
                   " l_thigh=" + name_or_idx(g_state.bone_l_thigh) +
                   " r_thigh=" + name_or_idx(g_state.bone_r_thigh) +
                   " l_calf=" + name_or_idx(g_state.bone_l_calf) +
                   " r_calf=" + name_or_idx(g_state.bone_r_calf) +
                   " l_foot=" + name_or_idx(g_state.bone_l_foot) +
                   " r_foot=" + name_or_idx(g_state.bone_r_foot) +
                   " l_arm=" + name_or_idx(g_state.bone_l_upper_arm) +
                   " r_arm=" + name_or_idx(g_state.bone_r_upper_arm) +
                   " l_fore=" + name_or_idx(g_state.bone_l_lower_arm) +
                   " r_fore=" + name_or_idx(g_state.bone_r_lower_arm));
    g_state.skinned_live_logged = true;
  }
  // TEMP: dump skeleton names once to validate bone mapping. Remove after mapping is confirmed.
  if (!g_state.skinned_live_dump_logged) {
    const size_t max_names = 64;
    std::string line = "renderer:vulkan skeleton names (first " + std::to_string(max_names) + "): ";
    for (size_t i = 0; i < g_state.skinned_skeleton.bones.size() && i < max_names; ++i) {
      const auto& name = g_state.skinned_skeleton.bones[i].name;
      if (!name.empty()) {
        line += std::to_string(i) + ":" + name + " ";
      }
    }
    if (g_state.skinned_skeleton.bones.size() > max_names) {
      line += "...";
    }
    rkg::log::info(line);
    g_state.skinned_live_dump_logged = true;
  }

  compute_skin_matrices_from_bones(posed, g_state.skinned_skeleton.inverse_bind, g_state.skinned_joint_mats);
  const size_t byte_count = g_state.skinned_joint_mats.size() * sizeof(rkg::Mat4);
  update_host_buffer(g_state.skinned_joint_mats.data(), byte_count, g_state.skinned_joint_memory);
}

#if RKG_ENABLE_DATA_JSON
using json = nlohmann::json;

bool read_vec3(const json& arr, float out[3]) {
  if (!arr.is_array() || arr.size() < 3) return false;
  out[0] = arr[0].get<float>();
  out[1] = arr[1].get<float>();
  out[2] = arr[2].get<float>();
  return true;
}

bool load_skeleton_json(const fs::path& path, SkeletonAsset& out) {
  std::ifstream in(path);
  if (!in) return false;
  json doc;
  try {
    in >> doc;
  } catch (...) {
    return false;
  }
  const auto bones = doc.value("bones", json::array());
  if (!bones.is_array()) return false;
  out.bones.clear();
  out.bones.reserve(bones.size());
  for (const auto& entry : bones) {
    if (!entry.is_object()) continue;
    BonePose bone{};
    bone.parent = entry.value("parent", -1);
    if (entry.contains("name") && entry["name"].is_string()) {
      bone.name = entry["name"].get<std::string>();
    }
    if (entry.contains("local_pose")) {
      const auto& pose = entry["local_pose"];
      if (pose.contains("position")) read_vec3(pose["position"], bone.pos);
      if (pose.contains("rotation")) read_vec3(pose["rotation"], bone.rot);
      if (pose.contains("scale")) read_vec3(pose["scale"], bone.scale);
    } else if (entry.contains("bind_local")) {
      const auto& bind = entry["bind_local"];
      if (bind.contains("position")) read_vec3(bind["position"], bone.pos);
      if (bind.contains("rotation")) read_vec3(bind["rotation"], bone.rot);
      if (bind.contains("scale")) read_vec3(bind["scale"], bone.scale);
    }
    out.bones.push_back(bone);
  }
  return !out.bones.empty();
}
#endif

bool load_skin_bin(const fs::path& path, SkeletonAsset& out) {
  std::vector<uint8_t> bytes;
  if (!read_file_bytes(path, bytes)) return false;
  if (bytes.size() < sizeof(uint32_t) * 4) return false;
  const uint32_t* header = reinterpret_cast<const uint32_t*>(bytes.data());
  const uint32_t magic = header[0];
  const uint32_t version = header[1];
  const uint32_t joint_count = header[2];
  if (magic != 0x4E494B53 || version != 1) return false;
  const size_t matrix_bytes = static_cast<size_t>(joint_count) * 16 * sizeof(float);
  const size_t offset = sizeof(uint32_t) * 4;
  if (bytes.size() < offset + matrix_bytes) return false;
  out.inverse_bind.resize(joint_count);
  const float* mats = reinterpret_cast<const float*>(bytes.data() + offset);
  for (uint32_t i = 0; i < joint_count; ++i) {
    rkg::Mat4 m{};
    std::memcpy(m.m, mats + i * 16, sizeof(float) * 16);
    out.inverse_bind[i] = m;
  }
  return true;
}

void compute_skin_matrices_from_bones(const std::vector<BonePose>& bones,
                                      const std::vector<rkg::Mat4>& inverse_bind,
                                      std::vector<rkg::Mat4>& out) {
  const size_t joint_count = bones.size();
  out.resize(joint_count);
  std::vector<rkg::Mat4> world(joint_count, rkg::mat4_identity());
  std::vector<uint8_t> visited(joint_count, 0);

  auto build_local = [](const BonePose& bone) {
    const rkg::Vec3 t{bone.pos[0], bone.pos[1], bone.pos[2]};
    const rkg::Vec3 r{bone.rot[0], bone.rot[1], bone.rot[2]};
    const rkg::Vec3 s{bone.scale[0], bone.scale[1], bone.scale[2]};
    return rkg::mat4_mul(rkg::mat4_translation(t),
                         rkg::mat4_mul(rkg::mat4_rotation_xyz(r), rkg::mat4_scale(s)));
  };

  std::function<rkg::Mat4(size_t)> compute_world = [&](size_t idx) -> rkg::Mat4 {
    if (idx >= joint_count) return rkg::mat4_identity();
    if (visited[idx]) return world[idx];
    visited[idx] = 1;
    const BonePose& bone = bones[idx];
    rkg::Mat4 local = build_local(bone);
    if (bone.parent >= 0 && static_cast<size_t>(bone.parent) < joint_count) {
      rkg::Mat4 parent_world = compute_world(static_cast<size_t>(bone.parent));
      world[idx] = rkg::mat4_mul(parent_world, local);
    } else {
      world[idx] = local;
    }
    return world[idx];
  };

  for (size_t i = 0; i < joint_count; ++i) {
    compute_world(i);
  }

  for (size_t i = 0; i < joint_count; ++i) {
    rkg::Mat4 inv = rkg::mat4_identity();
    if (i < inverse_bind.size()) inv = inverse_bind[i];
    out[i] = rkg::mat4_mul(world[i], inv);
  }
}

void compute_skin_matrices(const SkeletonAsset& skel, std::vector<rkg::Mat4>& out) {
  compute_skin_matrices_from_bones(skel.bones, skel.inverse_bind, out);
}

bool load_materials_json(const fs::path& path,
                         std::string& base_color_tex,
                         float base_color[4]) {
#if !RKG_ENABLE_DATA_JSON
  (void)path;
  (void)base_color_tex;
  (void)base_color;
  return false;
#else
  std::ifstream in(path);
  if (!in) return false;
  nlohmann::json doc;
  try {
    in >> doc;
  } catch (...) {
    return false;
  }
  const auto materials = doc.value("materials", nlohmann::json::array());
  if (!materials.empty()) {
    const auto& mat = materials.at(0);
    if (mat.contains("baseColorFactor") && mat["baseColorFactor"].is_array()) {
      const auto& arr = mat["baseColorFactor"];
      for (size_t i = 0; i < 4 && i < arr.size(); ++i) {
        base_color[i] = arr[i].get<float>();
      }
    }
    if (mat.contains("baseColorTexture") && mat["baseColorTexture"].is_string()) {
      base_color_tex = mat["baseColorTexture"].get<std::string>();
    }
  }
  return true;
#endif
}

bool load_textured_asset() {
  if (g_state.textured_ready) return true;
  if (g_state.device == VK_NULL_HANDLE) return false;

  std::string asset_name;
  const fs::path asset_dir = select_asset_dir(asset_name);
  if (asset_dir.empty()) {
    rkg::log::warn("renderer:vulkan textured demo: no asset directory found");
    return false;
  }
  g_state.textured_asset_name = asset_name;

  std::vector<TexturedVertex> vertices;
  std::vector<uint32_t> indices;
  bool has_uv = false;
  if (!load_mesh_bin_textured(asset_dir / "mesh.bin", vertices, indices, has_uv)) {
    rkg::log::warn("renderer:vulkan textured demo: failed to load mesh.bin");
    return false;
  }
  g_state.textured_has_uv0 = has_uv;

  if (!create_host_buffer(vertices.data(), vertices.size() * sizeof(TexturedVertex),
                          VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, g_state.textured_vertex_buffer,
                          g_state.textured_vertex_memory)) {
    rkg::log::warn("renderer:vulkan textured demo: vertex buffer create failed");
    return false;
  }
  if (!create_host_buffer(indices.data(), indices.size() * sizeof(uint32_t),
                          VK_BUFFER_USAGE_INDEX_BUFFER_BIT, g_state.textured_index_buffer,
                          g_state.textured_index_memory)) {
    rkg::log::warn("renderer:vulkan textured demo: index buffer create failed");
    return false;
  }
  g_state.textured_index_count = static_cast<uint32_t>(indices.size());

  // Reset to a matte grey each load so assets without materials don't inherit prior state.
  g_state.textured_base_color[0] = 0.65f;
  g_state.textured_base_color[1] = 0.65f;
  g_state.textured_base_color[2] = 0.65f;
  g_state.textured_base_color[3] = 1.0f;

  std::string base_color_tex;
  load_materials_json(asset_dir / "materials.json", base_color_tex, g_state.textured_base_color);

  std::vector<uint8_t> image_bytes;
  int tex_w = 1;
  int tex_h = 1;
  std::vector<uint8_t> rgba;
  if (!base_color_tex.empty()) {
    const fs::path tex_path = asset_dir / base_color_tex;
    if (read_file_bytes(tex_path, image_bytes)) {
      int w = 0, h = 0, ch = 0;
      stbi_uc* decoded = stbi_load_from_memory(image_bytes.data(),
                                               static_cast<int>(image_bytes.size()),
                                               &w, &h, &ch, 4);
      if (decoded) {
        tex_w = w;
        tex_h = h;
        rgba.assign(decoded, decoded + (w * h * 4));
        stbi_image_free(decoded);
      }
    } else {
      rkg::log::warn("renderer:vulkan textured demo: failed to read texture " + tex_path.string());
    }
  }
  if (rgba.empty()) {
    rgba.assign(4, 255);
    tex_w = 1;
    tex_h = 1;
  }
  if (!create_texture_from_rgba(rgba.data(), static_cast<uint32_t>(tex_w), static_cast<uint32_t>(tex_h))) {
    rkg::log::warn("renderer:vulkan textured demo: texture create failed");
    return false;
  }

  if (g_state.textured_desc_layout == VK_NULL_HANDLE) {
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo layout_info{};
    layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layout_info.bindingCount = 1;
    layout_info.pBindings = &binding;
    if (vkCreateDescriptorSetLayout(g_state.device, &layout_info, nullptr,
                                    &g_state.textured_desc_layout) != VK_SUCCESS) {
      rkg::log::warn("renderer:vulkan textured demo: descriptor layout create failed");
      return false;
    }
  }

  if (g_state.textured_desc_pool == VK_NULL_HANDLE) {
    VkDescriptorPoolSize pool_size{};
    pool_size.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    pool_size.descriptorCount = 1;
    VkDescriptorPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.poolSizeCount = 1;
    pool_info.pPoolSizes = &pool_size;
    pool_info.maxSets = 1;
    if (vkCreateDescriptorPool(g_state.device, &pool_info, nullptr, &g_state.textured_desc_pool) != VK_SUCCESS) {
      rkg::log::warn("renderer:vulkan textured demo: descriptor pool create failed");
      return false;
    }
  }

  if (g_state.textured_desc_set == VK_NULL_HANDLE) {
    VkDescriptorSetAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    alloc_info.descriptorPool = g_state.textured_desc_pool;
    alloc_info.descriptorSetCount = 1;
    alloc_info.pSetLayouts = &g_state.textured_desc_layout;
    if (vkAllocateDescriptorSets(g_state.device, &alloc_info, &g_state.textured_desc_set) != VK_SUCCESS) {
      rkg::log::warn("renderer:vulkan textured demo: descriptor set alloc failed");
      return false;
    }
  }

  VkDescriptorImageInfo image_info{};
  image_info.sampler = g_state.textured_sampler;
  image_info.imageView = g_state.textured_image_view;
  image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  VkWriteDescriptorSet write{};
  write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  write.dstSet = g_state.textured_desc_set;
  write.dstBinding = 0;
  write.descriptorCount = 1;
  write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  write.pImageInfo = &image_info;
  vkUpdateDescriptorSets(g_state.device, 1, &write, 0, nullptr);

  g_state.skinned_ready = false;
  g_state.skinned_index_count = 0;
  g_state.skinned_joint_count = 0;

  std::vector<SkinnedVertex> skinned_vertices;
  std::vector<uint32_t> skinned_indices;
  bool skinned_has_uv = false;
  if (load_mesh_bin_skinned(asset_dir / "mesh.bin", skinned_vertices, skinned_indices, skinned_has_uv)) {
    SkeletonAsset skeleton{};
#if RKG_ENABLE_DATA_JSON
    const bool skel_ok = load_skeleton_json(asset_dir / "skeleton.json", skeleton);
#else
    const bool skel_ok = false;
#endif
    const bool skin_ok = load_skin_bin(asset_dir / "skin.bin", skeleton);
    if (skel_ok && skin_ok && !skeleton.bones.empty()) {
      g_state.skinned_skeleton = skeleton;
      std::vector<rkg::Mat4> joint_mats;
      compute_skin_matrices(skeleton, joint_mats);
      if (!joint_mats.empty()) {
        if (!create_host_buffer(skinned_vertices.data(), skinned_vertices.size() * sizeof(SkinnedVertex),
                                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, g_state.skinned_vertex_buffer,
                                g_state.skinned_vertex_memory)) {
          rkg::log::warn("renderer:vulkan skinned demo: vertex buffer create failed");
          return false;
        }
        if (!create_host_buffer(skinned_indices.data(), skinned_indices.size() * sizeof(uint32_t),
                                VK_BUFFER_USAGE_INDEX_BUFFER_BIT, g_state.skinned_index_buffer,
                                g_state.skinned_index_memory)) {
          rkg::log::warn("renderer:vulkan skinned demo: index buffer create failed");
          return false;
        }
        if (!create_host_buffer(joint_mats.data(), joint_mats.size() * sizeof(rkg::Mat4),
                                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, g_state.skinned_joint_buffer,
                                g_state.skinned_joint_memory)) {
          rkg::log::warn("renderer:vulkan skinned demo: joint buffer create failed");
          return false;
        }
        g_state.skinned_joint_mats = joint_mats;
        g_state.skinned_index_count = static_cast<uint32_t>(skinned_indices.size());
        g_state.skinned_joint_count = static_cast<uint32_t>(joint_mats.size());

        if (g_state.skinned_desc_layout == VK_NULL_HANDLE) {
          VkDescriptorSetLayoutBinding bindings[2]{};
          bindings[0].binding = 0;
          bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
          bindings[0].descriptorCount = 1;
          bindings[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
          bindings[1].binding = 1;
          bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
          bindings[1].descriptorCount = 1;
          bindings[1].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
          VkDescriptorSetLayoutCreateInfo layout_info{};
          layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
          layout_info.bindingCount = 2;
          layout_info.pBindings = bindings;
          if (vkCreateDescriptorSetLayout(g_state.device, &layout_info, nullptr,
                                          &g_state.skinned_desc_layout) != VK_SUCCESS) {
            rkg::log::warn("renderer:vulkan skinned demo: descriptor layout create failed");
            return false;
          }
        }

        if (g_state.skinned_desc_pool == VK_NULL_HANDLE) {
          VkDescriptorPoolSize pool_sizes[2]{};
          pool_sizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
          pool_sizes[0].descriptorCount = 1;
          pool_sizes[1].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
          pool_sizes[1].descriptorCount = 1;
          VkDescriptorPoolCreateInfo pool_info{};
          pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
          pool_info.poolSizeCount = 2;
          pool_info.pPoolSizes = pool_sizes;
          pool_info.maxSets = 1;
          if (vkCreateDescriptorPool(g_state.device, &pool_info, nullptr, &g_state.skinned_desc_pool) != VK_SUCCESS) {
            rkg::log::warn("renderer:vulkan skinned demo: descriptor pool create failed");
            return false;
          }
        }

        if (g_state.skinned_desc_set == VK_NULL_HANDLE) {
          VkDescriptorSetAllocateInfo alloc_info{};
          alloc_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
          alloc_info.descriptorPool = g_state.skinned_desc_pool;
          alloc_info.descriptorSetCount = 1;
          alloc_info.pSetLayouts = &g_state.skinned_desc_layout;
          if (vkAllocateDescriptorSets(g_state.device, &alloc_info, &g_state.skinned_desc_set) != VK_SUCCESS) {
            rkg::log::warn("renderer:vulkan skinned demo: descriptor set alloc failed");
            return false;
          }
        }

        VkDescriptorImageInfo image_info{};
        image_info.sampler = g_state.textured_sampler;
        image_info.imageView = g_state.textured_image_view;
        image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkDescriptorBufferInfo buffer_info{};
        buffer_info.buffer = g_state.skinned_joint_buffer;
        buffer_info.offset = 0;
        buffer_info.range = joint_mats.size() * sizeof(rkg::Mat4);

        VkWriteDescriptorSet writes[2]{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = g_state.skinned_desc_set;
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[0].pImageInfo = &image_info;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = g_state.skinned_desc_set;
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[1].pBufferInfo = &buffer_info;
        vkUpdateDescriptorSets(g_state.device, 2, writes, 0, nullptr);

        g_state.skinned_ready = true;
        g_state.skinned_test_walk = env_flag_enabled("RKG_SKIN_TEST_WALK") ||
                                    rkg::get_vulkan_viewport_skinned_test_walk_enabled();
        g_state.skinned_test_walk_logged = false;
        g_state.skinned_live_map_valid = false;
        g_state.skinned_live_logged = false;
        rkg::log::info("renderer:vulkan skinned demo: joints=" +
                       std::to_string(g_state.skinned_joint_count));
      }
    }
  }

  g_state.textured_ready = true;
  rkg::log::info("renderer:vulkan textured demo: using asset " + g_state.textured_asset_name +
                 " dir=" + asset_dir.string());
  return true;
}

void destroy_offscreen() {
  if (g_state.textured_pipeline != VK_NULL_HANDLE) {
    vkDestroyPipeline(g_state.device, g_state.textured_pipeline, nullptr);
    g_state.textured_pipeline = VK_NULL_HANDLE;
  }
  if (g_state.textured_pipeline_translucent != VK_NULL_HANDLE) {
    vkDestroyPipeline(g_state.device, g_state.textured_pipeline_translucent, nullptr);
    g_state.textured_pipeline_translucent = VK_NULL_HANDLE;
  }
  if (g_state.skinned_pipeline != VK_NULL_HANDLE) {
    vkDestroyPipeline(g_state.device, g_state.skinned_pipeline, nullptr);
    g_state.skinned_pipeline = VK_NULL_HANDLE;
  }
  if (g_state.skinned_pipeline_translucent != VK_NULL_HANDLE) {
    vkDestroyPipeline(g_state.device, g_state.skinned_pipeline_translucent, nullptr);
    g_state.skinned_pipeline_translucent = VK_NULL_HANDLE;
  }
  if (g_state.viewport_pipeline != VK_NULL_HANDLE) {
    vkDestroyPipeline(g_state.device, g_state.viewport_pipeline, nullptr);
    g_state.viewport_pipeline = VK_NULL_HANDLE;
  }
  if (g_state.viewport_line_pipeline != VK_NULL_HANDLE) {
    vkDestroyPipeline(g_state.device, g_state.viewport_line_pipeline, nullptr);
    g_state.viewport_line_pipeline = VK_NULL_HANDLE;
  }
  if (g_state.viewport_pipeline_layout != VK_NULL_HANDLE) {
    vkDestroyPipelineLayout(g_state.device, g_state.viewport_pipeline_layout, nullptr);
    g_state.viewport_pipeline_layout = VK_NULL_HANDLE;
  }
  if (g_state.viewport_cube_buffer != VK_NULL_HANDLE) {
    vkDestroyBuffer(g_state.device, g_state.viewport_cube_buffer, nullptr);
    g_state.viewport_cube_buffer = VK_NULL_HANDLE;
  }
  if (g_state.viewport_cube_memory != VK_NULL_HANDLE) {
    vkFreeMemory(g_state.device, g_state.viewport_cube_memory, nullptr);
    g_state.viewport_cube_memory = VK_NULL_HANDLE;
  }
  if (g_state.viewport_quad_buffer != VK_NULL_HANDLE) {
    vkDestroyBuffer(g_state.device, g_state.viewport_quad_buffer, nullptr);
    g_state.viewport_quad_buffer = VK_NULL_HANDLE;
  }
  if (g_state.viewport_quad_memory != VK_NULL_HANDLE) {
    vkFreeMemory(g_state.device, g_state.viewport_quad_memory, nullptr);
    g_state.viewport_quad_memory = VK_NULL_HANDLE;
  }
  if (g_state.viewport_line_buffer != VK_NULL_HANDLE) {
    vkDestroyBuffer(g_state.device, g_state.viewport_line_buffer, nullptr);
    g_state.viewport_line_buffer = VK_NULL_HANDLE;
  }
  if (g_state.viewport_line_memory != VK_NULL_HANDLE) {
    vkFreeMemory(g_state.device, g_state.viewport_line_memory, nullptr);
    g_state.viewport_line_memory = VK_NULL_HANDLE;
  }
  g_state.viewport_cube_vertex_count = 0;
  g_state.viewport_quad_vertex_count = 0;
  g_state.viewport_line_vertex_capacity = 0;

  if (g_state.offscreen_depth_view != VK_NULL_HANDLE) {
    vkDestroyImageView(g_state.device, g_state.offscreen_depth_view, nullptr);
    g_state.offscreen_depth_view = VK_NULL_HANDLE;
  }
  if (g_state.offscreen_depth_image != VK_NULL_HANDLE) {
    vkDestroyImage(g_state.device, g_state.offscreen_depth_image, nullptr);
    g_state.offscreen_depth_image = VK_NULL_HANDLE;
  }
  if (g_state.offscreen_depth_memory != VK_NULL_HANDLE) {
    vkFreeMemory(g_state.device, g_state.offscreen_depth_memory, nullptr);
    g_state.offscreen_depth_memory = VK_NULL_HANDLE;
  }

  if (g_state.offscreen_framebuffer != VK_NULL_HANDLE) {
    vkDestroyFramebuffer(g_state.device, g_state.offscreen_framebuffer, nullptr);
    g_state.offscreen_framebuffer = VK_NULL_HANDLE;
  }
  if (g_state.offscreen_render_pass != VK_NULL_HANDLE) {
    vkDestroyRenderPass(g_state.device, g_state.offscreen_render_pass, nullptr);
    g_state.offscreen_render_pass = VK_NULL_HANDLE;
  }
  if (g_state.offscreen_view != VK_NULL_HANDLE) {
    vkDestroyImageView(g_state.device, g_state.offscreen_view, nullptr);
    g_state.offscreen_view = VK_NULL_HANDLE;
  }
  if (g_state.offscreen_sampler != VK_NULL_HANDLE) {
    vkDestroySampler(g_state.device, g_state.offscreen_sampler, nullptr);
    g_state.offscreen_sampler = VK_NULL_HANDLE;
  }
  if (g_state.offscreen_image != VK_NULL_HANDLE) {
    vkDestroyImage(g_state.device, g_state.offscreen_image, nullptr);
    g_state.offscreen_image = VK_NULL_HANDLE;
  }
  if (g_state.offscreen_memory != VK_NULL_HANDLE) {
    vkFreeMemory(g_state.device, g_state.offscreen_memory, nullptr);
    g_state.offscreen_memory = VK_NULL_HANDLE;
  }
  g_state.offscreen_extent = {};
  g_state.offscreen_color_layout = VK_IMAGE_LAYOUT_UNDEFINED;
  g_state.offscreen_depth_layout = VK_IMAGE_LAYOUT_UNDEFINED;
  rkg::register_vulkan_viewport(nullptr);
}

bool create_offscreen(uint32_t width, uint32_t height) {
  destroy_offscreen();
  if (width == 0 || height == 0) {
    return false;
  }

  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.format = g_state.swapchain_format;
  image_info.extent = {width, height, 1};
  image_info.mipLevels = 1;
  image_info.arrayLayers = 1;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

  if (vkCreateImage(g_state.device, &image_info, nullptr, &g_state.offscreen_image) != VK_SUCCESS) {
    rkg::log::error("offscreen image create failed");
    return false;
  }

  VkMemoryRequirements mem_req{};
  vkGetImageMemoryRequirements(g_state.device, g_state.offscreen_image, &mem_req);
  VkMemoryAllocateInfo alloc_info{};
  alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  alloc_info.allocationSize = mem_req.size;
  alloc_info.memoryTypeIndex = find_memory_type(mem_req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  if (vkAllocateMemory(g_state.device, &alloc_info, nullptr, &g_state.offscreen_memory) != VK_SUCCESS) {
    rkg::log::error("offscreen memory allocate failed");
    return false;
  }
  vkBindImageMemory(g_state.device, g_state.offscreen_image, g_state.offscreen_memory, 0);

  VkImageViewCreateInfo view_info{};
  view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_info.image = g_state.offscreen_image;
  view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  view_info.format = g_state.swapchain_format;
  view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  view_info.subresourceRange.baseMipLevel = 0;
  view_info.subresourceRange.levelCount = 1;
  view_info.subresourceRange.baseArrayLayer = 0;
  view_info.subresourceRange.layerCount = 1;
  if (vkCreateImageView(g_state.device, &view_info, nullptr, &g_state.offscreen_view) != VK_SUCCESS) {
    rkg::log::error("offscreen image view create failed");
    return false;
  }

  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
  if (vkCreateSampler(g_state.device, &sampler_info, nullptr, &g_state.offscreen_sampler) != VK_SUCCESS) {
    rkg::log::error("offscreen sampler create failed");
    return false;
  }

  VkImageCreateInfo depth_info{};
  depth_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  depth_info.imageType = VK_IMAGE_TYPE_2D;
  depth_info.format = g_state.offscreen_depth_format;
  depth_info.extent = {width, height, 1};
  depth_info.mipLevels = 1;
  depth_info.arrayLayers = 1;
  depth_info.samples = VK_SAMPLE_COUNT_1_BIT;
  depth_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  depth_info.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
  depth_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

  if (vkCreateImage(g_state.device, &depth_info, nullptr, &g_state.offscreen_depth_image) != VK_SUCCESS) {
    rkg::log::error("offscreen depth image create failed");
    return false;
  }

  VkMemoryRequirements depth_req{};
  vkGetImageMemoryRequirements(g_state.device, g_state.offscreen_depth_image, &depth_req);
  VkMemoryAllocateInfo depth_alloc{};
  depth_alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  depth_alloc.allocationSize = depth_req.size;
  depth_alloc.memoryTypeIndex = find_memory_type(depth_req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  if (vkAllocateMemory(g_state.device, &depth_alloc, nullptr, &g_state.offscreen_depth_memory) != VK_SUCCESS) {
    rkg::log::error("offscreen depth memory allocate failed");
    return false;
  }
  vkBindImageMemory(g_state.device, g_state.offscreen_depth_image, g_state.offscreen_depth_memory, 0);

  VkImageViewCreateInfo depth_view{};
  depth_view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  depth_view.image = g_state.offscreen_depth_image;
  depth_view.viewType = VK_IMAGE_VIEW_TYPE_2D;
  depth_view.format = g_state.offscreen_depth_format;
  depth_view.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
  depth_view.subresourceRange.baseMipLevel = 0;
  depth_view.subresourceRange.levelCount = 1;
  depth_view.subresourceRange.baseArrayLayer = 0;
  depth_view.subresourceRange.layerCount = 1;
  if (vkCreateImageView(g_state.device, &depth_view, nullptr, &g_state.offscreen_depth_view) != VK_SUCCESS) {
    rkg::log::error("offscreen depth view create failed");
    return false;
  }

  VkAttachmentDescription attachments[2]{};
  attachments[0].format = g_state.swapchain_format;
  attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
  attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  attachments[0].initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  // Keep the offscreen color attachment in COLOR_ATTACHMENT_OPTIMAL after the render pass.
  // We explicitly transition it to SHADER_READ_ONLY_OPTIMAL with a pipeline barrier below.
  attachments[0].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

  attachments[1].format = g_state.offscreen_depth_format;
  attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
  attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  attachments[1].initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
  attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

  VkAttachmentReference color_ref{};
  color_ref.attachment = 0;
  color_ref.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

  VkAttachmentReference depth_ref{};
  depth_ref.attachment = 1;
  depth_ref.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

  VkSubpassDescription subpass{};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachmentCount = 1;
  subpass.pColorAttachments = &color_ref;
  subpass.pDepthStencilAttachment = &depth_ref;

  VkRenderPassCreateInfo render_pass_info{};
  render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
  render_pass_info.attachmentCount = 2;
  render_pass_info.pAttachments = attachments;
  render_pass_info.subpassCount = 1;
  render_pass_info.pSubpasses = &subpass;

  if (vkCreateRenderPass(g_state.device, &render_pass_info, nullptr, &g_state.offscreen_render_pass) != VK_SUCCESS) {
    rkg::log::error("offscreen render pass create failed");
    return false;
  }

  VkImageView fb_attachments[] = {g_state.offscreen_view, g_state.offscreen_depth_view};
  VkFramebufferCreateInfo fb_info{};
  fb_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
  fb_info.renderPass = g_state.offscreen_render_pass;
  fb_info.attachmentCount = 2;
  fb_info.pAttachments = fb_attachments;
  fb_info.width = width;
  fb_info.height = height;
  fb_info.layers = 1;
  if (vkCreateFramebuffer(g_state.device, &fb_info, nullptr, &g_state.offscreen_framebuffer) != VK_SUCCESS) {
    rkg::log::error("offscreen framebuffer create failed");
    return false;
  }

  g_state.offscreen_extent = {width, height};
  g_state.offscreen_version += 1;
  g_state.offscreen_color_layout = VK_IMAGE_LAYOUT_UNDEFINED;
  g_state.offscreen_depth_layout = VK_IMAGE_LAYOUT_UNDEFINED;

  rkg::VulkanViewportHooks hooks;
  hooks.image_view = g_state.offscreen_view;
  hooks.sampler = g_state.offscreen_sampler;
  hooks.width = width;
  hooks.height = height;
  hooks.layout = static_cast<uint32_t>(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  hooks.version = g_state.offscreen_version;
  rkg::register_vulkan_viewport(&hooks);
  if (!create_viewport_pipeline()) {
    rkg::log::error("viewport pipeline setup failed");
    return false;
  }
  if (!create_viewport_line_pipeline()) {
    rkg::log::error("viewport line pipeline setup failed");
    return false;
  }
  if (!create_viewport_vertex_buffer()) {
    rkg::log::error("viewport vertex buffer setup failed");
    return false;
  }
  if (g_state.textured_ready) {
    if (!create_textured_pipeline()) {
      rkg::log::warn("textured pipeline setup failed");
    }
    if (!create_textured_pipeline_translucent()) {
      rkg::log::warn("textured translucent pipeline setup failed");
    }
  }
  if (g_state.skinned_ready) {
    if (!create_skinned_pipeline()) {
      rkg::log::warn("skinned pipeline setup failed");
    }
    if (!create_skinned_pipeline_translucent()) {
      rkg::log::warn("skinned translucent pipeline setup failed");
    }
  }

  return true;
}

void update_offscreen_target() {
  uint32_t req_w = 0;
  uint32_t req_h = 0;
  rkg::get_vulkan_viewport_request(req_w, req_h);
  if (req_w == 0 || req_h == 0) {
    if (g_state.offscreen_image != VK_NULL_HANDLE) {
      destroy_offscreen();
      g_state.skip_imgui_frame = true;
    }
    return;
  }
  if (g_state.offscreen_extent.width != req_w || g_state.offscreen_extent.height != req_h) {
    create_offscreen(req_w, req_h);
    g_state.skip_imgui_frame = true;
  }
}

VkShaderModule create_shader_module(const uint32_t* code, size_t word_count) {
  VkShaderModuleCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  create_info.codeSize = word_count * sizeof(uint32_t);
  create_info.pCode = code;
  VkShaderModule module = VK_NULL_HANDLE;
  if (vkCreateShaderModule(g_state.device, &create_info, nullptr, &module) != VK_SUCCESS) {
    return VK_NULL_HANDLE;
  }
  return module;
}

VkShaderModule create_shader_module_from_file(const fs::path& path) {
  std::vector<uint8_t> bytes;
  if (!read_file_bytes(path, bytes)) {
    return VK_NULL_HANDLE;
  }
  if (bytes.empty() || (bytes.size() % 4) != 0) {
    return VK_NULL_HANDLE;
  }
  VkShaderModuleCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  create_info.codeSize = bytes.size();
  create_info.pCode = reinterpret_cast<const uint32_t*>(bytes.data());
  VkShaderModule module = VK_NULL_HANDLE;
  if (vkCreateShaderModule(g_state.device, &create_info, nullptr, &module) != VK_SUCCESS) {
    return VK_NULL_HANDLE;
  }
  return module;
}

bool create_viewport_pipeline() {
  if (g_state.offscreen_render_pass == VK_NULL_HANDLE) return false;
  if (g_state.viewport_pipeline != VK_NULL_HANDLE) return true;

  VkShaderModule vert = create_shader_module(rkg_viewport_vert_spv,
                                             sizeof(rkg_viewport_vert_spv) / sizeof(uint32_t));
  VkShaderModule frag = create_shader_module(rkg_viewport_frag_spv,
                                             sizeof(rkg_viewport_frag_spv) / sizeof(uint32_t));
  if (vert == VK_NULL_HANDLE || frag == VK_NULL_HANDLE) {
    rkg::log::error("viewport shader module creation failed");
    if (vert != VK_NULL_HANDLE) vkDestroyShaderModule(g_state.device, vert, nullptr);
    if (frag != VK_NULL_HANDLE) vkDestroyShaderModule(g_state.device, frag, nullptr);
    return false;
  }

  VkPipelineShaderStageCreateInfo stages[2]{};
  stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
  stages[0].module = vert;
  stages[0].pName = "main";
  stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  stages[1].module = frag;
  stages[1].pName = "main";

  VkVertexInputBindingDescription binding{};
  binding.binding = 0;
  binding.stride = sizeof(float) * 3;
  binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

  VkVertexInputAttributeDescription attr{};
  attr.binding = 0;
  attr.location = 0;
  attr.format = VK_FORMAT_R32G32B32_SFLOAT;
  attr.offset = 0;

  VkPipelineVertexInputStateCreateInfo vertex_input{};
  vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
  vertex_input.vertexBindingDescriptionCount = 1;
  vertex_input.pVertexBindingDescriptions = &binding;
  vertex_input.vertexAttributeDescriptionCount = 1;
  vertex_input.pVertexAttributeDescriptions = &attr;

  VkPipelineInputAssemblyStateCreateInfo input_assembly{};
  input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  input_assembly.primitiveRestartEnable = VK_FALSE;

  VkPipelineViewportStateCreateInfo viewport_state{};
  viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
  viewport_state.viewportCount = 1;
  viewport_state.scissorCount = 1;

  VkPipelineRasterizationStateCreateInfo raster{};
  raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  raster.depthClampEnable = VK_FALSE;
  raster.rasterizerDiscardEnable = VK_FALSE;
  raster.polygonMode = VK_POLYGON_MODE_FILL;
  raster.lineWidth = 1.0f;
  raster.cullMode = VK_CULL_MODE_NONE;
  raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  raster.depthBiasEnable = VK_FALSE;

  VkPipelineMultisampleStateCreateInfo multisample{};
  multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

  VkPipelineDepthStencilStateCreateInfo depth{};
  depth.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
  // Depth test ON so grids/bones are occluded by meshes when appropriate.
  depth.depthTestEnable = VK_TRUE;
  depth.depthWriteEnable = VK_FALSE;
  depth.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
  depth.depthBoundsTestEnable = VK_FALSE;
  depth.stencilTestEnable = VK_FALSE;

  VkPipelineColorBlendAttachmentState color_blend_attach{};
  color_blend_attach.colorWriteMask =
      VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  color_blend_attach.blendEnable = VK_TRUE;
  color_blend_attach.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
  color_blend_attach.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  color_blend_attach.colorBlendOp = VK_BLEND_OP_ADD;
  color_blend_attach.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
  color_blend_attach.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  color_blend_attach.alphaBlendOp = VK_BLEND_OP_ADD;

  VkPipelineColorBlendStateCreateInfo color_blend{};
  color_blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  color_blend.attachmentCount = 1;
  color_blend.pAttachments = &color_blend_attach;

  VkDynamicState dynamics[] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
  VkPipelineDynamicStateCreateInfo dynamic{};
  dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
  dynamic.dynamicStateCount = 2;
  dynamic.pDynamicStates = dynamics;

  VkPushConstantRange push_range{};
  push_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
  push_range.offset = 0;
  push_range.size = sizeof(float) * 20;

  VkPipelineLayoutCreateInfo layout_info{};
  layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  layout_info.pushConstantRangeCount = 1;
  layout_info.pPushConstantRanges = &push_range;
  if (vkCreatePipelineLayout(g_state.device, &layout_info, nullptr, &g_state.viewport_pipeline_layout) != VK_SUCCESS) {
    rkg::log::error("viewport pipeline layout creation failed");
    vkDestroyShaderModule(g_state.device, vert, nullptr);
    vkDestroyShaderModule(g_state.device, frag, nullptr);
    return false;
  }

  VkGraphicsPipelineCreateInfo pipeline_info{};
  pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  pipeline_info.stageCount = 2;
  pipeline_info.pStages = stages;
  pipeline_info.pVertexInputState = &vertex_input;
  pipeline_info.pInputAssemblyState = &input_assembly;
  pipeline_info.pViewportState = &viewport_state;
  pipeline_info.pRasterizationState = &raster;
  pipeline_info.pMultisampleState = &multisample;
  pipeline_info.pDepthStencilState = &depth;
  pipeline_info.pColorBlendState = &color_blend;
  pipeline_info.pDynamicState = &dynamic;
  pipeline_info.layout = g_state.viewport_pipeline_layout;
  pipeline_info.renderPass = g_state.offscreen_render_pass;
  pipeline_info.subpass = 0;

  const VkResult result = vkCreateGraphicsPipelines(g_state.device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr,
                                                    &g_state.viewport_pipeline);
  vkDestroyShaderModule(g_state.device, vert, nullptr);
  vkDestroyShaderModule(g_state.device, frag, nullptr);
  if (result != VK_SUCCESS) {
    rkg::log::error("viewport pipeline creation failed");
    return false;
  }
  return true;
}

bool create_viewport_line_pipeline() {
  if (g_state.offscreen_render_pass == VK_NULL_HANDLE) return false;
  if (g_state.viewport_line_pipeline != VK_NULL_HANDLE) return true;
  if (g_state.viewport_pipeline_layout == VK_NULL_HANDLE) {
    if (!create_viewport_pipeline()) {
      return false;
    }
  }

  VkShaderModule vert = create_shader_module(rkg_viewport_vert_spv,
                                             sizeof(rkg_viewport_vert_spv) / sizeof(uint32_t));
  VkShaderModule frag = create_shader_module(rkg_viewport_frag_spv,
                                             sizeof(rkg_viewport_frag_spv) / sizeof(uint32_t));
  if (vert == VK_NULL_HANDLE || frag == VK_NULL_HANDLE) {
    rkg::log::error("viewport line shader module creation failed");
    if (vert != VK_NULL_HANDLE) vkDestroyShaderModule(g_state.device, vert, nullptr);
    if (frag != VK_NULL_HANDLE) vkDestroyShaderModule(g_state.device, frag, nullptr);
    return false;
  }

  VkPipelineShaderStageCreateInfo stages[2]{};
  stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
  stages[0].module = vert;
  stages[0].pName = "main";
  stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  stages[1].module = frag;
  stages[1].pName = "main";

  VkVertexInputBindingDescription binding{};
  binding.binding = 0;
  binding.stride = sizeof(float) * 3;
  binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

  VkVertexInputAttributeDescription attr{};
  attr.binding = 0;
  attr.location = 0;
  attr.format = VK_FORMAT_R32G32B32_SFLOAT;
  attr.offset = 0;

  VkPipelineVertexInputStateCreateInfo vertex_input{};
  vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
  vertex_input.vertexBindingDescriptionCount = 1;
  vertex_input.pVertexBindingDescriptions = &binding;
  vertex_input.vertexAttributeDescriptionCount = 1;
  vertex_input.pVertexAttributeDescriptions = &attr;

  VkPipelineInputAssemblyStateCreateInfo input_assembly{};
  input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
  input_assembly.primitiveRestartEnable = VK_FALSE;

  VkPipelineViewportStateCreateInfo viewport_state{};
  viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
  viewport_state.viewportCount = 1;
  viewport_state.scissorCount = 1;

  VkPipelineRasterizationStateCreateInfo raster{};
  raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  raster.depthClampEnable = VK_FALSE;
  raster.rasterizerDiscardEnable = VK_FALSE;
  raster.polygonMode = VK_POLYGON_MODE_FILL;
  raster.lineWidth = 1.0f;
  raster.cullMode = VK_CULL_MODE_NONE;
  raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  raster.depthBiasEnable = VK_FALSE;

  VkPipelineMultisampleStateCreateInfo multisample{};
  multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

  VkPipelineDepthStencilStateCreateInfo depth{};
  depth.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
  depth.depthTestEnable = VK_TRUE;
  depth.depthWriteEnable = VK_FALSE;
  depth.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
  depth.depthBoundsTestEnable = VK_FALSE;
  depth.stencilTestEnable = VK_FALSE;

  VkPipelineColorBlendAttachmentState color_blend_attach{};
  color_blend_attach.colorWriteMask =
      VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  color_blend_attach.blendEnable = VK_TRUE;
  color_blend_attach.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
  color_blend_attach.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  color_blend_attach.colorBlendOp = VK_BLEND_OP_ADD;
  color_blend_attach.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
  color_blend_attach.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  color_blend_attach.alphaBlendOp = VK_BLEND_OP_ADD;

  VkPipelineColorBlendStateCreateInfo color_blend{};
  color_blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  color_blend.attachmentCount = 1;
  color_blend.pAttachments = &color_blend_attach;

  VkDynamicState dynamics[] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
  VkPipelineDynamicStateCreateInfo dynamic{};
  dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
  dynamic.dynamicStateCount = 2;
  dynamic.pDynamicStates = dynamics;

  VkGraphicsPipelineCreateInfo pipeline_info{};
  pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  pipeline_info.stageCount = 2;
  pipeline_info.pStages = stages;
  pipeline_info.pVertexInputState = &vertex_input;
  pipeline_info.pInputAssemblyState = &input_assembly;
  pipeline_info.pViewportState = &viewport_state;
  pipeline_info.pRasterizationState = &raster;
  pipeline_info.pMultisampleState = &multisample;
  pipeline_info.pDepthStencilState = &depth;
  pipeline_info.pColorBlendState = &color_blend;
  pipeline_info.pDynamicState = &dynamic;
  pipeline_info.layout = g_state.viewport_pipeline_layout;
  pipeline_info.renderPass = g_state.offscreen_render_pass;
  pipeline_info.subpass = 0;

  const VkResult result = vkCreateGraphicsPipelines(g_state.device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr,
                                                    &g_state.viewport_line_pipeline);
  vkDestroyShaderModule(g_state.device, vert, nullptr);
  vkDestroyShaderModule(g_state.device, frag, nullptr);
  if (result != VK_SUCCESS) {
    rkg::log::error("viewport line pipeline creation failed");
    return false;
  }
  return true;
}

static bool create_textured_pipeline_internal(VkPipeline* out_pipeline, bool depth_write, bool enable_blend) {
  if (g_state.offscreen_render_pass == VK_NULL_HANDLE) return false;
  if (!out_pipeline) return false;
  if (*out_pipeline != VK_NULL_HANDLE) return true;

  if (g_state.textured_desc_layout == VK_NULL_HANDLE) {
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo layout_info{};
    layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layout_info.bindingCount = 1;
    layout_info.pBindings = &binding;
    if (vkCreateDescriptorSetLayout(g_state.device, &layout_info, nullptr,
                                    &g_state.textured_desc_layout) != VK_SUCCESS) {
      rkg::log::error("textured pipeline descriptor layout create failed");
      return false;
    }
  }

  if (g_state.textured_pipeline_layout == VK_NULL_HANDLE) {
    VkPushConstantRange push_range{};
    push_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    push_range.offset = 0;
    push_range.size = sizeof(float) * 20;
    VkPipelineLayoutCreateInfo layout_info{};
    layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layout_info.setLayoutCount = 1;
    layout_info.pSetLayouts = &g_state.textured_desc_layout;
    layout_info.pushConstantRangeCount = 1;
    layout_info.pPushConstantRanges = &push_range;
    if (vkCreatePipelineLayout(g_state.device, &layout_info, nullptr,
                               &g_state.textured_pipeline_layout) != VK_SUCCESS) {
      rkg::log::error("textured pipeline layout creation failed");
      return false;
    }
  }

  const fs::path shader_dir = rkg::resolve_paths(nullptr, std::nullopt, "demo_game").root /
                              "plugins/renderer/vulkan/shaders";
  VkShaderModule vert = create_shader_module_from_file(shader_dir / "mesh_textured.vert.spv");
  VkShaderModule frag = create_shader_module_from_file(shader_dir / "mesh_textured.frag.spv");
  if (vert == VK_NULL_HANDLE || frag == VK_NULL_HANDLE) {
    rkg::log::error("textured shader module creation failed");
    if (vert != VK_NULL_HANDLE) vkDestroyShaderModule(g_state.device, vert, nullptr);
    if (frag != VK_NULL_HANDLE) vkDestroyShaderModule(g_state.device, frag, nullptr);
    return false;
  }

  VkPipelineShaderStageCreateInfo stages[2]{};
  stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
  stages[0].module = vert;
  stages[0].pName = "main";
  stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  stages[1].module = frag;
  stages[1].pName = "main";

  VkVertexInputBindingDescription binding{};
  binding.binding = 0;
  binding.stride = sizeof(TexturedVertex);
  binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

  VkVertexInputAttributeDescription attrs[2]{};
  attrs[0].binding = 0;
  attrs[0].location = 0;
  attrs[0].format = VK_FORMAT_R32G32B32_SFLOAT;
  attrs[0].offset = 0;
  attrs[1].binding = 0;
  attrs[1].location = 1;
  attrs[1].format = VK_FORMAT_R32G32_SFLOAT;
  attrs[1].offset = sizeof(float) * 3;

  VkPipelineVertexInputStateCreateInfo vertex_input{};
  vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
  vertex_input.vertexBindingDescriptionCount = 1;
  vertex_input.pVertexBindingDescriptions = &binding;
  vertex_input.vertexAttributeDescriptionCount = 2;
  vertex_input.pVertexAttributeDescriptions = attrs;

  VkPipelineInputAssemblyStateCreateInfo input_assembly{};
  input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  input_assembly.primitiveRestartEnable = VK_FALSE;

  VkPipelineViewportStateCreateInfo viewport_state{};
  viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
  viewport_state.viewportCount = 1;
  viewport_state.scissorCount = 1;

  VkPipelineRasterizationStateCreateInfo raster{};
  raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  raster.depthClampEnable = VK_FALSE;
  raster.rasterizerDiscardEnable = VK_FALSE;
  raster.polygonMode = VK_POLYGON_MODE_FILL;
  raster.lineWidth = 1.0f;
  raster.cullMode = VK_CULL_MODE_NONE;
  raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  raster.depthBiasEnable = VK_FALSE;

  VkPipelineMultisampleStateCreateInfo multisample{};
  multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

  VkPipelineDepthStencilStateCreateInfo depth{};
  depth.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
  depth.depthTestEnable = VK_TRUE;
  depth.depthWriteEnable = depth_write ? VK_TRUE : VK_FALSE;
  depth.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
  depth.depthBoundsTestEnable = VK_FALSE;
  depth.stencilTestEnable = VK_FALSE;

  VkPipelineColorBlendAttachmentState color_blend_attach{};
  color_blend_attach.colorWriteMask =
      VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  color_blend_attach.blendEnable = enable_blend ? VK_TRUE : VK_FALSE;
  if (enable_blend) {
    color_blend_attach.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    color_blend_attach.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    color_blend_attach.colorBlendOp = VK_BLEND_OP_ADD;
    color_blend_attach.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    color_blend_attach.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    color_blend_attach.alphaBlendOp = VK_BLEND_OP_ADD;
  }

  VkPipelineColorBlendStateCreateInfo color_blend{};
  color_blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  color_blend.attachmentCount = 1;
  color_blend.pAttachments = &color_blend_attach;

  VkDynamicState dynamics[] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
  VkPipelineDynamicStateCreateInfo dynamic{};
  dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
  dynamic.dynamicStateCount = 2;
  dynamic.pDynamicStates = dynamics;

  VkGraphicsPipelineCreateInfo pipeline_info{};
  pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  pipeline_info.stageCount = 2;
  pipeline_info.pStages = stages;
  pipeline_info.pVertexInputState = &vertex_input;
  pipeline_info.pInputAssemblyState = &input_assembly;
  pipeline_info.pViewportState = &viewport_state;
  pipeline_info.pRasterizationState = &raster;
  pipeline_info.pMultisampleState = &multisample;
  pipeline_info.pDepthStencilState = &depth;
  pipeline_info.pColorBlendState = &color_blend;
  pipeline_info.pDynamicState = &dynamic;
  pipeline_info.layout = g_state.textured_pipeline_layout;
  pipeline_info.renderPass = g_state.offscreen_render_pass;
  pipeline_info.subpass = 0;

  const VkResult result = vkCreateGraphicsPipelines(g_state.device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr,
                                                    out_pipeline);
  vkDestroyShaderModule(g_state.device, vert, nullptr);
  vkDestroyShaderModule(g_state.device, frag, nullptr);
  if (result != VK_SUCCESS) {
    rkg::log::error("textured pipeline creation failed");
    return false;
  }
  return true;
}

bool create_textured_pipeline() {
  return create_textured_pipeline_internal(&g_state.textured_pipeline, true, false);
}

bool create_textured_pipeline_translucent() {
  return create_textured_pipeline_internal(&g_state.textured_pipeline_translucent, false, true);
}

static bool create_skinned_pipeline_internal(VkPipeline* out_pipeline, bool depth_write, bool enable_blend) {
  if (g_state.offscreen_render_pass == VK_NULL_HANDLE) return false;
  if (!out_pipeline) return false;
  if (*out_pipeline != VK_NULL_HANDLE) return true;

  if (g_state.skinned_desc_layout == VK_NULL_HANDLE) {
    rkg::log::error("skinned pipeline descriptor layout missing");
    return false;
  }

  if (g_state.skinned_pipeline_layout == VK_NULL_HANDLE) {
    VkPushConstantRange push_range{};
    push_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    push_range.offset = 0;
    push_range.size = sizeof(float) * 20;
    VkPipelineLayoutCreateInfo layout_info{};
    layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layout_info.setLayoutCount = 1;
    layout_info.pSetLayouts = &g_state.skinned_desc_layout;
    layout_info.pushConstantRangeCount = 1;
    layout_info.pPushConstantRanges = &push_range;
    if (vkCreatePipelineLayout(g_state.device, &layout_info, nullptr,
                               &g_state.skinned_pipeline_layout) != VK_SUCCESS) {
      rkg::log::error("skinned pipeline layout creation failed");
      return false;
    }
  }

  const fs::path shader_dir = rkg::resolve_paths(nullptr, std::nullopt, "demo_game").root /
                              "plugins/renderer/vulkan/shaders";
  VkShaderModule vert = create_shader_module_from_file(shader_dir / "mesh_skinned.vert.spv");
  VkShaderModule frag = create_shader_module_from_file(shader_dir / "mesh_skinned.frag.spv");
  if (vert == VK_NULL_HANDLE || frag == VK_NULL_HANDLE) {
    rkg::log::error("skinned shader module creation failed");
    if (vert != VK_NULL_HANDLE) vkDestroyShaderModule(g_state.device, vert, nullptr);
    if (frag != VK_NULL_HANDLE) vkDestroyShaderModule(g_state.device, frag, nullptr);
    return false;
  }

  VkPipelineShaderStageCreateInfo stages[2]{};
  stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
  stages[0].module = vert;
  stages[0].pName = "main";
  stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  stages[1].module = frag;
  stages[1].pName = "main";

  VkVertexInputBindingDescription binding{};
  binding.binding = 0;
  binding.stride = sizeof(SkinnedVertex);
  binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

  VkVertexInputAttributeDescription attrs[4]{};
  attrs[0].binding = 0;
  attrs[0].location = 0;
  attrs[0].format = VK_FORMAT_R32G32B32_SFLOAT;
  attrs[0].offset = offsetof(SkinnedVertex, pos);
  attrs[1].binding = 0;
  attrs[1].location = 1;
  attrs[1].format = VK_FORMAT_R32G32_SFLOAT;
  attrs[1].offset = offsetof(SkinnedVertex, uv);
  attrs[2].binding = 0;
  attrs[2].location = 2;
  attrs[2].format = VK_FORMAT_R16G16B16A16_UINT;
  attrs[2].offset = offsetof(SkinnedVertex, joints);
  attrs[3].binding = 0;
  attrs[3].location = 3;
  attrs[3].format = VK_FORMAT_R32G32B32A32_SFLOAT;
  attrs[3].offset = offsetof(SkinnedVertex, weights);

  VkPipelineVertexInputStateCreateInfo vertex_input{};
  vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
  vertex_input.vertexBindingDescriptionCount = 1;
  vertex_input.pVertexBindingDescriptions = &binding;
  vertex_input.vertexAttributeDescriptionCount = 4;
  vertex_input.pVertexAttributeDescriptions = attrs;

  VkPipelineInputAssemblyStateCreateInfo input_assembly{};
  input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  input_assembly.primitiveRestartEnable = VK_FALSE;

  VkPipelineViewportStateCreateInfo viewport_state{};
  viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
  viewport_state.viewportCount = 1;
  viewport_state.scissorCount = 1;

  VkPipelineRasterizationStateCreateInfo raster{};
  raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  raster.depthClampEnable = VK_FALSE;
  raster.rasterizerDiscardEnable = VK_FALSE;
  raster.polygonMode = VK_POLYGON_MODE_FILL;
  raster.lineWidth = 1.0f;
  raster.cullMode = VK_CULL_MODE_NONE;
  raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  raster.depthBiasEnable = VK_FALSE;

  VkPipelineMultisampleStateCreateInfo multisample{};
  multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

  VkPipelineDepthStencilStateCreateInfo depth{};
  depth.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
  depth.depthTestEnable = VK_TRUE;
  depth.depthWriteEnable = depth_write ? VK_TRUE : VK_FALSE;
  depth.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
  depth.depthBoundsTestEnable = VK_FALSE;
  depth.stencilTestEnable = VK_FALSE;

  VkPipelineColorBlendAttachmentState color_blend_attach{};
  color_blend_attach.colorWriteMask =
      VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  color_blend_attach.blendEnable = enable_blend ? VK_TRUE : VK_FALSE;
  if (enable_blend) {
    color_blend_attach.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    color_blend_attach.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    color_blend_attach.colorBlendOp = VK_BLEND_OP_ADD;
    color_blend_attach.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    color_blend_attach.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    color_blend_attach.alphaBlendOp = VK_BLEND_OP_ADD;
  }

  VkPipelineColorBlendStateCreateInfo color_blend{};
  color_blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  color_blend.attachmentCount = 1;
  color_blend.pAttachments = &color_blend_attach;

  VkDynamicState dynamics[] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
  VkPipelineDynamicStateCreateInfo dynamic{};
  dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
  dynamic.dynamicStateCount = 2;
  dynamic.pDynamicStates = dynamics;

  VkGraphicsPipelineCreateInfo pipeline_info{};
  pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  pipeline_info.stageCount = 2;
  pipeline_info.pStages = stages;
  pipeline_info.pVertexInputState = &vertex_input;
  pipeline_info.pInputAssemblyState = &input_assembly;
  pipeline_info.pViewportState = &viewport_state;
  pipeline_info.pRasterizationState = &raster;
  pipeline_info.pMultisampleState = &multisample;
  pipeline_info.pDepthStencilState = &depth;
  pipeline_info.pColorBlendState = &color_blend;
  pipeline_info.pDynamicState = &dynamic;
  pipeline_info.layout = g_state.skinned_pipeline_layout;
  pipeline_info.renderPass = g_state.offscreen_render_pass;
  pipeline_info.subpass = 0;

  const VkResult result = vkCreateGraphicsPipelines(g_state.device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr,
                                                    out_pipeline);
  vkDestroyShaderModule(g_state.device, vert, nullptr);
  vkDestroyShaderModule(g_state.device, frag, nullptr);
  if (result != VK_SUCCESS) {
    rkg::log::error("skinned pipeline creation failed");
    return false;
  }
  return true;
}

bool create_skinned_pipeline() {
  return create_skinned_pipeline_internal(&g_state.skinned_pipeline, true, false);
}

bool create_skinned_pipeline_translucent() {
  return create_skinned_pipeline_internal(&g_state.skinned_pipeline_translucent, false, true);
}

bool create_viewport_vertex_buffer() {
  if (g_state.viewport_cube_buffer != VK_NULL_HANDLE && g_state.viewport_quad_buffer != VK_NULL_HANDLE &&
      g_state.viewport_line_buffer != VK_NULL_HANDLE) {
    return true;
  }

  auto create_buffer = [&](const float* data, size_t byte_size, VkBuffer& buffer, VkDeviceMemory& memory) -> bool {
    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = static_cast<VkDeviceSize>(byte_size);
    buffer_info.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateBuffer(g_state.device, &buffer_info, nullptr, &buffer) != VK_SUCCESS) {
      return false;
    }

    VkMemoryRequirements mem_req{};
    vkGetBufferMemoryRequirements(g_state.device, buffer, &mem_req);
    VkMemoryAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.allocationSize = mem_req.size;
    alloc_info.memoryTypeIndex = find_memory_type(
        mem_req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    if (vkAllocateMemory(g_state.device, &alloc_info, nullptr, &memory) != VK_SUCCESS) {
      return false;
    }
    vkBindBufferMemory(g_state.device, buffer, memory, 0);

    void* mapped = nullptr;
    if (vkMapMemory(g_state.device, memory, 0, byte_size, 0, &mapped) != VK_SUCCESS) {
      return false;
    }
    std::memcpy(mapped, data, byte_size);
    vkUnmapMemory(g_state.device, memory);
    return true;
  };

  if (g_state.viewport_cube_buffer == VK_NULL_HANDLE) {
    const size_t cube_bytes = sizeof(kViewportCubeVertices);
    if (!create_buffer(kViewportCubeVertices, cube_bytes, g_state.viewport_cube_buffer, g_state.viewport_cube_memory)) {
      rkg::log::error("viewport cube buffer create failed");
      return false;
    }
    g_state.viewport_cube_vertex_count = kViewportVertexCount;
  }

  if (g_state.viewport_quad_buffer == VK_NULL_HANDLE) {
    const size_t quad_bytes = sizeof(kViewportQuadVertices);
    if (!create_buffer(kViewportQuadVertices, quad_bytes, g_state.viewport_quad_buffer, g_state.viewport_quad_memory)) {
      rkg::log::error("viewport quad buffer create failed");
      return false;
    }
    g_state.viewport_quad_vertex_count = kViewportQuadVertexCount;
  }

  if (g_state.viewport_line_buffer == VK_NULL_HANDLE) {
    const size_t line_bytes =
        static_cast<size_t>(rkg::VulkanViewportLineList::kMaxLines) * 2 * 3 * sizeof(float);
    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = static_cast<VkDeviceSize>(line_bytes);
    buffer_info.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(g_state.device, &buffer_info, nullptr, &g_state.viewport_line_buffer) != VK_SUCCESS) {
      rkg::log::error("viewport line buffer create failed");
      return false;
    }
    VkMemoryRequirements mem_req{};
    vkGetBufferMemoryRequirements(g_state.device, g_state.viewport_line_buffer, &mem_req);
    VkMemoryAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.allocationSize = mem_req.size;
    alloc_info.memoryTypeIndex = find_memory_type(
        mem_req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    if (vkAllocateMemory(g_state.device, &alloc_info, nullptr, &g_state.viewport_line_memory) != VK_SUCCESS) {
      rkg::log::error("viewport line memory alloc failed");
      return false;
    }
    vkBindBufferMemory(g_state.device, g_state.viewport_line_buffer, g_state.viewport_line_memory, 0);
    g_state.viewport_line_vertex_capacity = rkg::VulkanViewportLineList::kMaxLines * 2;
  }

  return true;
}

VkSurfaceFormatKHR choose_surface_format(const std::vector<VkSurfaceFormatKHR>& formats) {
  for (const auto& fmt : formats) {
    if (fmt.format == VK_FORMAT_B8G8R8A8_SRGB && fmt.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
      return fmt;
    }
  }
  return formats[0];
}

VkPresentModeKHR choose_present_mode(const std::vector<VkPresentModeKHR>& modes) {
  for (const auto& mode : modes) {
    if (mode == VK_PRESENT_MODE_MAILBOX_KHR) {
      return mode;
    }
  }
  return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D choose_extent(const VkSurfaceCapabilitiesKHR& caps) {
  if (caps.currentExtent.width != UINT32_MAX) {
    return caps.currentExtent;
  }
  int w = 0;
  int h = 0;
  get_window_drawable_size(w, h);
  VkExtent2D extent{};
  extent.width = static_cast<uint32_t>(w);
  extent.height = static_cast<uint32_t>(h);
  extent.width = std::max(caps.minImageExtent.width, std::min(caps.maxImageExtent.width, extent.width));
  extent.height = std::max(caps.minImageExtent.height, std::min(caps.maxImageExtent.height, extent.height));
  return extent;
}

bool create_swapchain() {
  VkSurfaceCapabilitiesKHR caps{};
  vkGetPhysicalDeviceSurfaceCapabilitiesKHR(g_state.physical_device, g_state.surface, &caps);

  uint32_t format_count = 0;
  vkGetPhysicalDeviceSurfaceFormatsKHR(g_state.physical_device, g_state.surface, &format_count, nullptr);
  std::vector<VkSurfaceFormatKHR> formats(format_count);
  vkGetPhysicalDeviceSurfaceFormatsKHR(g_state.physical_device, g_state.surface, &format_count, formats.data());

  uint32_t present_mode_count = 0;
  vkGetPhysicalDeviceSurfacePresentModesKHR(g_state.physical_device, g_state.surface, &present_mode_count, nullptr);
  std::vector<VkPresentModeKHR> present_modes(present_mode_count);
  vkGetPhysicalDeviceSurfacePresentModesKHR(g_state.physical_device, g_state.surface, &present_mode_count,
                                            present_modes.data());

  VkSurfaceFormatKHR surface_format = choose_surface_format(formats);
  VkPresentModeKHR present_mode = choose_present_mode(present_modes);
  VkExtent2D extent = choose_extent(caps);
  if (extent.width == 0 || extent.height == 0) {
    g_state.swapchain_needs_rebuild = true;
    rkg::log::warn("renderer:vulkan swapchain extent is 0; create_swapchain deferred");
    return false;
  }

  uint32_t image_count = caps.minImageCount + 1;
  if (caps.maxImageCount > 0 && image_count > caps.maxImageCount) {
    image_count = caps.maxImageCount;
  }

  VkSwapchainCreateInfoKHR create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
  create_info.surface = g_state.surface;
  create_info.minImageCount = image_count;
  create_info.imageFormat = surface_format.format;
  create_info.imageColorSpace = surface_format.colorSpace;
  create_info.imageExtent = extent;
  create_info.imageArrayLayers = 1;
  create_info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

  uint32_t queue_family_indices[] = {g_state.graphics_queue_family, g_state.present_queue_family};
  if (g_state.graphics_queue_family != g_state.present_queue_family) {
    create_info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
    create_info.queueFamilyIndexCount = 2;
    create_info.pQueueFamilyIndices = queue_family_indices;
  } else {
    create_info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
  }

  create_info.preTransform = caps.currentTransform;
  create_info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
  create_info.presentMode = present_mode;
  create_info.clipped = VK_TRUE;

  if (vkCreateSwapchainKHR(g_state.device, &create_info, nullptr, &g_state.swapchain) != VK_SUCCESS) {
    rkg::log::error("vkCreateSwapchainKHR failed");
    return false;
  }

  g_state.swapchain_format = surface_format.format;
  g_state.swapchain_extent = extent;

  uint32_t swapchain_image_count = 0;
  vkGetSwapchainImagesKHR(g_state.device, g_state.swapchain, &swapchain_image_count, nullptr);
  g_state.swapchain_images.resize(swapchain_image_count);
  vkGetSwapchainImagesKHR(g_state.device, g_state.swapchain, &swapchain_image_count,
                          g_state.swapchain_images.data());

  g_state.swapchain_image_views.resize(swapchain_image_count);
  for (size_t i = 0; i < g_state.swapchain_images.size(); ++i) {
    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = g_state.swapchain_images[i];
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = g_state.swapchain_format;
    view_info.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
    view_info.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
    view_info.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
    view_info.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.baseArrayLayer = 0;
    view_info.subresourceRange.layerCount = 1;

    if (vkCreateImageView(g_state.device, &view_info, nullptr, &g_state.swapchain_image_views[i]) != VK_SUCCESS) {
      rkg::log::error("vkCreateImageView failed");
      return false;
    }
  }
  g_state.swapchain_image_layouts.assign(g_state.swapchain_images.size(), VK_IMAGE_LAYOUT_UNDEFINED);

  return true;
}

bool create_render_pass() {
  VkAttachmentDescription color_attachment{};
  color_attachment.format = g_state.swapchain_format;
  color_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
  color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  color_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  color_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  color_attachment.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  color_attachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

  VkAttachmentReference color_attachment_ref{};
  color_attachment_ref.attachment = 0;
  color_attachment_ref.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

  VkSubpassDescription subpass{};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachmentCount = 1;
  subpass.pColorAttachments = &color_attachment_ref;

  VkSubpassDependency dependency{};
  dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
  dependency.dstSubpass = 0;
  dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

  VkRenderPassCreateInfo render_pass_info{};
  render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
  render_pass_info.attachmentCount = 1;
  render_pass_info.pAttachments = &color_attachment;
  render_pass_info.subpassCount = 1;
  render_pass_info.pSubpasses = &subpass;
  render_pass_info.dependencyCount = 1;
  render_pass_info.pDependencies = &dependency;

  if (vkCreateRenderPass(g_state.device, &render_pass_info, nullptr, &g_state.render_pass) != VK_SUCCESS) {
    rkg::log::error("vkCreateRenderPass failed");
    return false;
  }
  return true;
}

bool recreate_swapchain() {
  if (g_state.device == VK_NULL_HANDLE) {
    return false;
  }
  if (g_state.surface_lost) {
    if (g_state.surface != VK_NULL_HANDLE) {
      vkDestroySurfaceKHR(g_state.instance, g_state.surface, nullptr);
      g_state.surface = VK_NULL_HANDLE;
    }
    if (!create_surface()) {
      rkg::log::error("renderer:vulkan recreate surface failed");
      return false;
    }
    g_state.surface_lost = false;
  }
  if (g_state.surface == VK_NULL_HANDLE) {
    return false;
  }
  // Ensure no pending work is using the swapchain before destroying it.
  if (g_state.device != VK_NULL_HANDLE) {
    vkDeviceWaitIdle(g_state.device);
  }
  const VkFormat old_format = g_state.swapchain_format;
  const uint32_t old_image_count = static_cast<uint32_t>(g_state.swapchain_images.size());
  VkRenderPass old_render_pass = g_state.render_pass;
  destroy_swapchain(true);
  if (!create_swapchain()) {
    if (g_state.swapchain_needs_rebuild) {
      rkg::log::warn("renderer:vulkan swapchain not ready; deferring init");
      return true;
    }
    return false;
  }
  if (old_format != VK_FORMAT_UNDEFINED && g_state.swapchain_format != old_format) {
    rkg::log::info("renderer:vulkan swapchain format changed; rebuilding offscreen resources");
    destroy_offscreen();
  }
  bool render_pass_changed = false;
  bool image_count_changed = false;
  if (old_render_pass == VK_NULL_HANDLE || g_state.swapchain_format != old_format) {
    if (old_render_pass != VK_NULL_HANDLE) {
      vkDestroyRenderPass(g_state.device, old_render_pass, nullptr);
    }
    g_state.render_pass = VK_NULL_HANDLE;
  }
  if (g_state.render_pass == VK_NULL_HANDLE) {
    if (!create_render_pass()) return false;
    render_pass_changed = true;
  }
  image_count_changed = old_image_count != static_cast<uint32_t>(g_state.swapchain_images.size());
  if (!create_framebuffers()) return false;
  if (g_state.command_pool != VK_NULL_HANDLE) {
    vkResetCommandPool(g_state.device, g_state.command_pool, 0);
  }
  if (!create_command_buffers()) return false;

  rkg::VulkanHooks hooks;
  hooks.instance = g_state.instance;
  hooks.physical_device = g_state.physical_device;
  hooks.device = g_state.device;
  hooks.queue = g_state.graphics_queue;
  hooks.render_pass = g_state.render_pass;
  hooks.window = g_state.window;
  hooks.queue_family = g_state.graphics_queue_family;
  hooks.image_count = static_cast<uint32_t>(g_state.swapchain_images.size());
  hooks.swapchain_format = static_cast<uint32_t>(g_state.swapchain_format);
  rkg::register_vulkan_hooks(&hooks);
#if RKG_ENABLE_IMGUI
  if ((render_pass_changed || image_count_changed) && rkg::debug_ui::is_initialized()) {
    rkg::log::info("debug_ui: reinit after swapchain change");
    rkg::debug_ui::shutdown();
    if (!rkg::debug_ui::init_vulkan()) {
      rkg::log::warn("debug_ui: reinit failed after swapchain change");
    }
  }
#endif

  return true;
}

bool create_framebuffers() {
  g_state.framebuffers.resize(g_state.swapchain_image_views.size());
  for (size_t i = 0; i < g_state.swapchain_image_views.size(); ++i) {
    VkImageView attachments[] = {g_state.swapchain_image_views[i]};
    VkFramebufferCreateInfo framebuffer_info{};
    framebuffer_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    framebuffer_info.renderPass = g_state.render_pass;
    framebuffer_info.attachmentCount = 1;
    framebuffer_info.pAttachments = attachments;
    framebuffer_info.width = g_state.swapchain_extent.width;
    framebuffer_info.height = g_state.swapchain_extent.height;
    framebuffer_info.layers = 1;

    if (vkCreateFramebuffer(g_state.device, &framebuffer_info, nullptr, &g_state.framebuffers[i]) != VK_SUCCESS) {
      rkg::log::error("vkCreateFramebuffer failed");
      return false;
    }
  }
  return true;
}

bool create_command_pool() {
  VkCommandPoolCreateInfo pool_info{};
  pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  pool_info.queueFamilyIndex = g_state.graphics_queue_family;
  pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

  if (vkCreateCommandPool(g_state.device, &pool_info, nullptr, &g_state.command_pool) != VK_SUCCESS) {
    rkg::log::error("vkCreateCommandPool failed");
    return false;
  }
  return true;
}

bool create_command_buffers() {
  g_state.command_buffers.resize(g_state.framebuffers.size());
  VkCommandBufferAllocateInfo alloc_info{};
  alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  alloc_info.commandPool = g_state.command_pool;
  alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  alloc_info.commandBufferCount = static_cast<uint32_t>(g_state.command_buffers.size());

  if (vkAllocateCommandBuffers(g_state.device, &alloc_info, g_state.command_buffers.data()) != VK_SUCCESS) {
    rkg::log::error("vkAllocateCommandBuffers failed");
    return false;
  }
  return true;
}

bool create_sync_objects() {
  VkSemaphoreCreateInfo sem_info{};
  sem_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

  if (vkCreateSemaphore(g_state.device, &sem_info, nullptr, &g_state.image_available) != VK_SUCCESS) {
    rkg::log::error("vkCreateSemaphore failed");
    return false;
  }
  if (vkCreateSemaphore(g_state.device, &sem_info, nullptr, &g_state.render_finished) != VK_SUCCESS) {
    rkg::log::error("vkCreateSemaphore failed");
    return false;
  }

  VkFenceCreateInfo fence_info{};
  fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  fence_info.flags = VK_FENCE_CREATE_SIGNALED_BIT;
  if (vkCreateFence(g_state.device, &fence_info, nullptr, &g_state.in_flight_fence) != VK_SUCCESS) {
    rkg::log::error("vkCreateFence failed");
    return false;
  }

  return true;
}

void destroy_swapchain(bool keep_render_pass) {
  for (auto fb : g_state.framebuffers) {
    vkDestroyFramebuffer(g_state.device, fb, nullptr);
  }
  g_state.framebuffers.clear();

  if (!keep_render_pass && g_state.render_pass != VK_NULL_HANDLE) {
    vkDestroyRenderPass(g_state.device, g_state.render_pass, nullptr);
    g_state.render_pass = VK_NULL_HANDLE;
  }

  for (auto view : g_state.swapchain_image_views) {
    vkDestroyImageView(g_state.device, view, nullptr);
  }
  g_state.swapchain_image_views.clear();
  g_state.swapchain_image_layouts.clear();

  if (g_state.swapchain != VK_NULL_HANDLE) {
    vkDestroySwapchainKHR(g_state.device, g_state.swapchain, nullptr);
    g_state.swapchain = VK_NULL_HANDLE;
  }
}

void destroy_vulkan() {
  if (g_state.device != VK_NULL_HANDLE) {
    vkDeviceWaitIdle(g_state.device);
  }

  destroy_offscreen();

  if (g_state.textured_sampler != VK_NULL_HANDLE) {
    vkDestroySampler(g_state.device, g_state.textured_sampler, nullptr);
    g_state.textured_sampler = VK_NULL_HANDLE;
  }
  if (g_state.textured_image_view != VK_NULL_HANDLE) {
    vkDestroyImageView(g_state.device, g_state.textured_image_view, nullptr);
    g_state.textured_image_view = VK_NULL_HANDLE;
  }
  if (g_state.textured_image != VK_NULL_HANDLE) {
    vkDestroyImage(g_state.device, g_state.textured_image, nullptr);
    g_state.textured_image = VK_NULL_HANDLE;
  }
  if (g_state.textured_image_memory != VK_NULL_HANDLE) {
    vkFreeMemory(g_state.device, g_state.textured_image_memory, nullptr);
    g_state.textured_image_memory = VK_NULL_HANDLE;
  }
  if (g_state.textured_vertex_buffer != VK_NULL_HANDLE) {
    vkDestroyBuffer(g_state.device, g_state.textured_vertex_buffer, nullptr);
    g_state.textured_vertex_buffer = VK_NULL_HANDLE;
  }
  if (g_state.textured_vertex_memory != VK_NULL_HANDLE) {
    vkFreeMemory(g_state.device, g_state.textured_vertex_memory, nullptr);
    g_state.textured_vertex_memory = VK_NULL_HANDLE;
  }
  if (g_state.textured_index_buffer != VK_NULL_HANDLE) {
    vkDestroyBuffer(g_state.device, g_state.textured_index_buffer, nullptr);
    g_state.textured_index_buffer = VK_NULL_HANDLE;
  }
  if (g_state.textured_index_memory != VK_NULL_HANDLE) {
    vkFreeMemory(g_state.device, g_state.textured_index_memory, nullptr);
    g_state.textured_index_memory = VK_NULL_HANDLE;
  }
  if (g_state.textured_desc_pool != VK_NULL_HANDLE) {
    vkDestroyDescriptorPool(g_state.device, g_state.textured_desc_pool, nullptr);
    g_state.textured_desc_pool = VK_NULL_HANDLE;
  }
  if (g_state.textured_desc_layout != VK_NULL_HANDLE) {
    vkDestroyDescriptorSetLayout(g_state.device, g_state.textured_desc_layout, nullptr);
    g_state.textured_desc_layout = VK_NULL_HANDLE;
  }
  if (g_state.textured_pipeline_layout != VK_NULL_HANDLE) {
    vkDestroyPipelineLayout(g_state.device, g_state.textured_pipeline_layout, nullptr);
    g_state.textured_pipeline_layout = VK_NULL_HANDLE;
  }
  if (g_state.skinned_joint_buffer != VK_NULL_HANDLE) {
    vkDestroyBuffer(g_state.device, g_state.skinned_joint_buffer, nullptr);
    g_state.skinned_joint_buffer = VK_NULL_HANDLE;
  }
  if (g_state.skinned_joint_memory != VK_NULL_HANDLE) {
    vkFreeMemory(g_state.device, g_state.skinned_joint_memory, nullptr);
    g_state.skinned_joint_memory = VK_NULL_HANDLE;
  }
  if (g_state.skinned_vertex_buffer != VK_NULL_HANDLE) {
    vkDestroyBuffer(g_state.device, g_state.skinned_vertex_buffer, nullptr);
    g_state.skinned_vertex_buffer = VK_NULL_HANDLE;
  }
  if (g_state.skinned_vertex_memory != VK_NULL_HANDLE) {
    vkFreeMemory(g_state.device, g_state.skinned_vertex_memory, nullptr);
    g_state.skinned_vertex_memory = VK_NULL_HANDLE;
  }
  if (g_state.skinned_index_buffer != VK_NULL_HANDLE) {
    vkDestroyBuffer(g_state.device, g_state.skinned_index_buffer, nullptr);
    g_state.skinned_index_buffer = VK_NULL_HANDLE;
  }
  if (g_state.skinned_index_memory != VK_NULL_HANDLE) {
    vkFreeMemory(g_state.device, g_state.skinned_index_memory, nullptr);
    g_state.skinned_index_memory = VK_NULL_HANDLE;
  }
  if (g_state.skinned_desc_pool != VK_NULL_HANDLE) {
    vkDestroyDescriptorPool(g_state.device, g_state.skinned_desc_pool, nullptr);
    g_state.skinned_desc_pool = VK_NULL_HANDLE;
  }
  if (g_state.skinned_desc_layout != VK_NULL_HANDLE) {
    vkDestroyDescriptorSetLayout(g_state.device, g_state.skinned_desc_layout, nullptr);
    g_state.skinned_desc_layout = VK_NULL_HANDLE;
  }
  if (g_state.skinned_pipeline_layout != VK_NULL_HANDLE) {
    vkDestroyPipelineLayout(g_state.device, g_state.skinned_pipeline_layout, nullptr);
    g_state.skinned_pipeline_layout = VK_NULL_HANDLE;
  }

  if (g_state.in_flight_fence != VK_NULL_HANDLE) {
    vkDestroyFence(g_state.device, g_state.in_flight_fence, nullptr);
  }
  if (g_state.image_available != VK_NULL_HANDLE) {
    vkDestroySemaphore(g_state.device, g_state.image_available, nullptr);
  }
  if (g_state.render_finished != VK_NULL_HANDLE) {
    vkDestroySemaphore(g_state.device, g_state.render_finished, nullptr);
  }
  if (g_state.command_pool != VK_NULL_HANDLE) {
    vkDestroyCommandPool(g_state.device, g_state.command_pool, nullptr);
  }

  destroy_swapchain();

  if (g_state.device != VK_NULL_HANDLE) {
    vkDestroyDevice(g_state.device, nullptr);
  }
  if (g_state.surface != VK_NULL_HANDLE) {
    vkDestroySurfaceKHR(g_state.instance, g_state.surface, nullptr);
  }
  if (g_state.instance != VK_NULL_HANDLE) {
    vkDestroyInstance(g_state.instance, nullptr);
  }
}

bool record_command_buffer(VkCommandBuffer cmd, uint32_t image_index) {
  log_step("vkBeginCommandBuffer");
  VkCommandBufferBeginInfo begin_info{};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  const VkResult begin_result = vkBeginCommandBuffer(cmd, &begin_info);
  if (begin_result != VK_SUCCESS) {
    rkg::log::error(std::string("renderer:vulkan vkBeginCommandBuffer failed: ") +
                    vk_result_name(begin_result) + " (" + std::to_string(begin_result) + ")");
    return false;
  }

  if (g_state.offscreen_render_pass != VK_NULL_HANDLE && g_state.offscreen_framebuffer != VK_NULL_HANDLE) {
    VkImageMemoryBarrier offscreen_color_to_attachment{};
    offscreen_color_to_attachment.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    offscreen_color_to_attachment.srcAccessMask = 0;
    offscreen_color_to_attachment.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    offscreen_color_to_attachment.oldLayout = g_state.offscreen_color_layout;
    offscreen_color_to_attachment.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    offscreen_color_to_attachment.image = g_state.offscreen_image;
    offscreen_color_to_attachment.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    offscreen_color_to_attachment.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    offscreen_color_to_attachment.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    offscreen_color_to_attachment.subresourceRange.baseMipLevel = 0;
    offscreen_color_to_attachment.subresourceRange.levelCount = 1;
    offscreen_color_to_attachment.subresourceRange.baseArrayLayer = 0;
    offscreen_color_to_attachment.subresourceRange.layerCount = 1;

    VkImageMemoryBarrier offscreen_depth_to_attachment{};
    offscreen_depth_to_attachment.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    offscreen_depth_to_attachment.srcAccessMask = 0;
    offscreen_depth_to_attachment.dstAccessMask =
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    offscreen_depth_to_attachment.oldLayout = g_state.offscreen_depth_layout;
    offscreen_depth_to_attachment.newLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    offscreen_depth_to_attachment.image = g_state.offscreen_depth_image;
    offscreen_depth_to_attachment.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    offscreen_depth_to_attachment.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    offscreen_depth_to_attachment.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    offscreen_depth_to_attachment.subresourceRange.baseMipLevel = 0;
    offscreen_depth_to_attachment.subresourceRange.levelCount = 1;
    offscreen_depth_to_attachment.subresourceRange.baseArrayLayer = 0;
    offscreen_depth_to_attachment.subresourceRange.layerCount = 1;

    VkImageMemoryBarrier offscreen_barriers[] = {offscreen_color_to_attachment, offscreen_depth_to_attachment};
    log_step("offscreen: pipeline barrier to attachment");
    vkCmdPipelineBarrier(cmd,
                         VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                         VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                             VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
                             VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
                         0,
                         0,
                         nullptr,
                         0,
                         nullptr,
                         2,
                         offscreen_barriers);

    g_state.offscreen_color_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    g_state.offscreen_depth_layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkClearValue offscreen_clears[2]{};
    offscreen_clears[0].color = {{0.08f, 0.08f, 0.12f, 1.0f}};
    offscreen_clears[1].depthStencil = {1.0f, 0};

    VkRenderPassBeginInfo offscreen_info{};
    offscreen_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    offscreen_info.renderPass = g_state.offscreen_render_pass;
    offscreen_info.framebuffer = g_state.offscreen_framebuffer;
    offscreen_info.renderArea.offset = {0, 0};
    offscreen_info.renderArea.extent = g_state.offscreen_extent;
    offscreen_info.clearValueCount = 2;
    offscreen_info.pClearValues = offscreen_clears;

    log_step("offscreen: begin render pass");
    vkCmdBeginRenderPass(cmd, &offscreen_info, VK_SUBPASS_CONTENTS_INLINE);
    const auto* draw_list = rkg::get_vulkan_viewport_draw_list();
    if (g_state.viewport_pipeline != VK_NULL_HANDLE && g_state.viewport_pipeline_layout != VK_NULL_HANDLE &&
        g_state.viewport_cube_buffer != VK_NULL_HANDLE && g_state.viewport_cube_vertex_count > 0 &&
        draw_list && draw_list->instance_count > 0) {
      VkViewport viewport{};
      viewport.x = 0.0f;
      viewport.y = 0.0f;
      viewport.width = static_cast<float>(g_state.offscreen_extent.width);
      viewport.height = static_cast<float>(g_state.offscreen_extent.height);
      viewport.minDepth = 0.0f;
      viewport.maxDepth = 1.0f;
      VkRect2D scissor{};
      scissor.offset = {0, 0};
      scissor.extent = g_state.offscreen_extent;
      vkCmdSetViewport(cmd, 0, 1, &viewport);
      vkCmdSetScissor(cmd, 0, 1, &scissor);
      vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, g_state.viewport_pipeline);
      const uint32_t max_instances =
          std::min(draw_list->instance_count, rkg::VulkanViewportDrawList::kMaxInstances);
      for (uint32_t i = 0; i < max_instances; ++i) {
        VkBuffer buffer = g_state.viewport_cube_buffer;
        uint32_t vertex_count = g_state.viewport_cube_vertex_count;
        if (draw_list->mesh_id[i] == 1 && g_state.viewport_quad_buffer != VK_NULL_HANDLE &&
            g_state.viewport_quad_vertex_count > 0) {
          buffer = g_state.viewport_quad_buffer;
          vertex_count = g_state.viewport_quad_vertex_count;
        }
        VkDeviceSize offsets[] = {0};
        vkCmdBindVertexBuffers(cmd, 0, 1, &buffer, offsets);

        struct PushData {
          float mvp[16];
          float color[4];
        } push{};
        const float* mvp = draw_list->mvp + (i * 16);
        const float* color = draw_list->color + (i * 4);
        std::memcpy(push.mvp, mvp, sizeof(push.mvp));
        std::memcpy(push.color, color, sizeof(push.color));
        vkCmdPushConstants(cmd, g_state.viewport_pipeline_layout,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushData), &push);
        vkCmdDraw(cmd, vertex_count, 1, 0, 0);
      }
    }
    const auto* camera = rkg::get_vulkan_viewport_camera();
    update_skinned_live_pose();
    const auto* textured_demo = rkg::get_vulkan_viewport_textured_demo();
    const bool textured_enabled = rkg::get_vulkan_viewport_textured_demo_enabled();
    const bool use_skinned = g_state.skinned_ready &&
                             g_state.skinned_pipeline != VK_NULL_HANDLE &&
                             g_state.skinned_pipeline_layout != VK_NULL_HANDLE &&
                             g_state.skinned_vertex_buffer != VK_NULL_HANDLE &&
                             g_state.skinned_index_buffer != VK_NULL_HANDLE &&
                             g_state.skinned_index_count > 0;
    if (textured_enabled &&
        g_state.textured_ready &&
        ((use_skinned) ||
         (g_state.textured_pipeline != VK_NULL_HANDLE &&
          g_state.textured_pipeline_layout != VK_NULL_HANDLE &&
          g_state.textured_vertex_buffer != VK_NULL_HANDLE &&
          g_state.textured_index_buffer != VK_NULL_HANDLE &&
          g_state.textured_index_count > 0))) {
      VkViewport viewport{};
      viewport.x = 0.0f;
      viewport.y = 0.0f;
      viewport.width = static_cast<float>(g_state.offscreen_extent.width);
      viewport.height = static_cast<float>(g_state.offscreen_extent.height);
      viewport.minDepth = 0.0f;
      viewport.maxDepth = 1.0f;
      VkRect2D scissor{};
      scissor.offset = {0, 0};
      scissor.extent = g_state.offscreen_extent;
      vkCmdSetViewport(cmd, 0, 1, &viewport);
      vkCmdSetScissor(cmd, 0, 1, &scissor);

      const float textured_alpha = textured_demo ? textured_demo->alpha : 1.0f;
      const bool translucent = textured_alpha < 0.999f;
      if (use_skinned) {
        VkPipeline pipeline = translucent ? g_state.skinned_pipeline_translucent : g_state.skinned_pipeline;
        if (pipeline == VK_NULL_HANDLE) pipeline = g_state.skinned_pipeline;
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
      } else {
        VkPipeline pipeline = translucent ? g_state.textured_pipeline_translucent : g_state.textured_pipeline;
        if (pipeline == VK_NULL_HANDLE) pipeline = g_state.textured_pipeline;
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
      }
      VkDeviceSize offsets[] = {0};
      if (use_skinned) {
        vkCmdBindVertexBuffers(cmd, 0, 1, &g_state.skinned_vertex_buffer, offsets);
        vkCmdBindIndexBuffer(cmd, g_state.skinned_index_buffer, 0, VK_INDEX_TYPE_UINT32);
        if (g_state.skinned_desc_set != VK_NULL_HANDLE) {
          vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                  g_state.skinned_pipeline_layout, 0, 1,
                                  &g_state.skinned_desc_set, 0, nullptr);
        }
      } else {
        vkCmdBindVertexBuffers(cmd, 0, 1, &g_state.textured_vertex_buffer, offsets);
        vkCmdBindIndexBuffer(cmd, g_state.textured_index_buffer, 0, VK_INDEX_TYPE_UINT32);
        if (g_state.textured_desc_set != VK_NULL_HANDLE) {
          vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                  g_state.textured_pipeline_layout, 0, 1,
                                  &g_state.textured_desc_set, 0, nullptr);
        }
      }

      struct PushData {
        float mvp[16];
        float color[4];
      } push{};
      if (textured_demo && textured_demo->has_mvp) {
        std::memcpy(push.mvp, textured_demo->mvp, sizeof(push.mvp));
      } else if (camera) {
        std::memcpy(push.mvp, camera->view_proj, sizeof(push.mvp));
      } else {
        std::memset(push.mvp, 0, sizeof(push.mvp));
        push.mvp[0] = 1.0f;
        push.mvp[5] = 1.0f;
        push.mvp[10] = 1.0f;
        push.mvp[15] = 1.0f;
      }
      std::memcpy(push.color, g_state.textured_base_color, sizeof(push.color));
      if (textured_demo) {
        push.color[3] *= textured_demo->alpha;
      }
      if (use_skinned) {
        vkCmdPushConstants(cmd, g_state.skinned_pipeline_layout,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                           0, sizeof(PushData), &push);
        vkCmdDrawIndexed(cmd, g_state.skinned_index_count, 1, 0, 0, 0);
      } else {
        vkCmdPushConstants(cmd, g_state.textured_pipeline_layout,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                           0, sizeof(PushData), &push);
        vkCmdDrawIndexed(cmd, g_state.textured_index_count, 1, 0, 0, 0);
      }
    }

    const auto* line_list = rkg::get_vulkan_viewport_line_list();
    if (camera && line_list && line_list->line_count > 0 &&
        g_state.viewport_line_pipeline != VK_NULL_HANDLE &&
        g_state.viewport_line_buffer != VK_NULL_HANDLE &&
        g_state.viewport_pipeline_layout != VK_NULL_HANDLE) {
      VkViewport viewport{};
      viewport.x = 0.0f;
      viewport.y = 0.0f;
      viewport.width = static_cast<float>(g_state.offscreen_extent.width);
      viewport.height = static_cast<float>(g_state.offscreen_extent.height);
      viewport.minDepth = 0.0f;
      viewport.maxDepth = 1.0f;
      VkRect2D scissor{};
      scissor.offset = {0, 0};
      scissor.extent = g_state.offscreen_extent;
      vkCmdSetViewport(cmd, 0, 1, &viewport);
      vkCmdSetScissor(cmd, 0, 1, &scissor);
      const uint32_t max_lines =
          std::min(line_list->line_count, rkg::VulkanViewportLineList::kMaxLines);
      const size_t vertex_count = static_cast<size_t>(max_lines) * 2;
      if (vertex_count <= g_state.viewport_line_vertex_capacity) {
        void* mapped = nullptr;
        const size_t byte_count = static_cast<size_t>(max_lines) * 6 * sizeof(float);
        if (vkMapMemory(g_state.device, g_state.viewport_line_memory, 0, byte_count, 0, &mapped) == VK_SUCCESS) {
          std::memcpy(mapped, line_list->positions, byte_count);
          vkUnmapMemory(g_state.device, g_state.viewport_line_memory);
        }

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, g_state.viewport_line_pipeline);
        VkBuffer line_buffer = g_state.viewport_line_buffer;
        VkDeviceSize offsets[] = {0};
        vkCmdBindVertexBuffers(cmd, 0, 1, &line_buffer, offsets);

        struct PushData {
          float mvp[16];
          float color[4];
        } push{};
        std::memcpy(push.mvp, camera->view_proj, sizeof(push.mvp));
        for (uint32_t i = 0; i < max_lines; ++i) {
          const float* color = line_list->colors + (i * 4);
          std::memcpy(push.color, color, sizeof(push.color));
          vkCmdPushConstants(cmd, g_state.viewport_pipeline_layout,
                             VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushData), &push);
          const uint32_t first_vertex = i * 2;
          vkCmdDraw(cmd, 2, 1, first_vertex, 0);
        }
      }
    }
    log_step("offscreen: end render pass");
    vkCmdEndRenderPass(cmd);

    // Ensure offscreen color attachment writes are visible to fragment sampling (ImGui viewport).
    VkImageMemoryBarrier offscreen_barrier{};
    offscreen_barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    offscreen_barrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    offscreen_barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    offscreen_barrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    offscreen_barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    offscreen_barrier.image = g_state.offscreen_image;
    offscreen_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    offscreen_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    offscreen_barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    offscreen_barrier.subresourceRange.baseMipLevel = 0;
    offscreen_barrier.subresourceRange.levelCount = 1;
    offscreen_barrier.subresourceRange.baseArrayLayer = 0;
    offscreen_barrier.subresourceRange.layerCount = 1;
    log_step("offscreen: barrier to shader read");
    vkCmdPipelineBarrier(cmd,
                         VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                         VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                         0,
                         0,
                         nullptr,
                         0,
                         nullptr,
                         1,
                         &offscreen_barrier);
    g_state.offscreen_color_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  }

#if RKG_ENABLE_IMGUI
  const bool render_inside = rkg::debug_ui::render_inside_pass();
#else
  const bool render_inside = true;
#endif

  if (render_inside) {
    VkImageLayout old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
    if (image_index < g_state.swapchain_image_layouts.size()) {
      old_layout = g_state.swapchain_image_layouts[image_index];
    }

    VkImageMemoryBarrier to_color{};
    to_color.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    to_color.srcAccessMask = 0;
    to_color.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    to_color.oldLayout = old_layout;
    to_color.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    to_color.image = g_state.swapchain_images[image_index];
    to_color.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_color.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_color.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    to_color.subresourceRange.baseMipLevel = 0;
    to_color.subresourceRange.levelCount = 1;
    to_color.subresourceRange.baseArrayLayer = 0;
    to_color.subresourceRange.layerCount = 1;
    log_step("swapchain: barrier to color attachment");
    vkCmdPipelineBarrier(cmd,
                         VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                         VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                         0,
                         0,
                         nullptr,
                         0,
                         nullptr,
                         1,
                         &to_color);
    if (image_index < g_state.swapchain_image_layouts.size()) {
      g_state.swapchain_image_layouts[image_index] = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    }

    VkClearValue clear_color{};
    clear_color.color = {{0.02f, 0.02f, 0.05f, 1.0f}};

    VkRenderPassBeginInfo render_pass_info{};
    render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    render_pass_info.renderPass = g_state.render_pass;
    render_pass_info.framebuffer = g_state.framebuffers[image_index];
    render_pass_info.renderArea.offset = {0, 0};
    render_pass_info.renderArea.extent = g_state.swapchain_extent;
    render_pass_info.clearValueCount = 1;
    render_pass_info.pClearValues = &clear_color;

    log_step("swapchain: begin render pass");
    vkCmdBeginRenderPass(cmd, &render_pass_info, VK_SUBPASS_CONTENTS_INLINE);

    VkClearAttachment clear_attachment{};
    clear_attachment.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    clear_attachment.colorAttachment = 0;
    clear_attachment.clearValue.color = {{0.15f, 0.65f, 0.95f, 1.0f}};

    VkClearRect rect{};
    rect.baseArrayLayer = 0;
    rect.layerCount = 1;
    rect.rect.offset = {static_cast<int32_t>(g_state.swapchain_extent.width / 4),
                        static_cast<int32_t>(g_state.swapchain_extent.height / 4)};
    rect.rect.extent = {g_state.swapchain_extent.width / 2, g_state.swapchain_extent.height / 2};

    log_step("swapchain: clear attachments");
    vkCmdClearAttachments(cmd, 1, &clear_attachment, 1, &rect);

#if RKG_ENABLE_IMGUI
    if (g_state.skip_imgui_frame) {
      log_step("imgui: skip render (viewport resized)");
      g_state.skip_imgui_frame = false;
    } else {
      log_step("imgui: render inside pass");
      rkg::debug_ui::render(cmd);
    }
#endif
    log_step("swapchain: end render pass");
    vkCmdEndRenderPass(cmd);
    if (image_index < g_state.swapchain_image_layouts.size()) {
      g_state.swapchain_image_layouts[image_index] = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    }
  }

#if RKG_ENABLE_IMGUI
  if (!render_inside) {
    if (!g_state.dynamic_rendering_enabled) {
      rkg::log::warn("debug_ui render skipped: dynamic rendering not enabled");
      return vkEndCommandBuffer(cmd) == VK_SUCCESS;
    }

    VkImageLayout old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
    if (image_index < g_state.swapchain_image_layouts.size()) {
      old_layout = g_state.swapchain_image_layouts[image_index];
    }

    VkImageMemoryBarrier to_color{};
    to_color.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    to_color.srcAccessMask = 0;
    to_color.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    to_color.oldLayout = old_layout;
    to_color.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    to_color.image = g_state.swapchain_images[image_index];
    to_color.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_color.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_color.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    to_color.subresourceRange.baseMipLevel = 0;
    to_color.subresourceRange.levelCount = 1;
    to_color.subresourceRange.baseArrayLayer = 0;
    to_color.subresourceRange.layerCount = 1;
    log_step("dynamic rendering: barrier to color attachment");
    vkCmdPipelineBarrier(cmd,
                         VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                         VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                         0,
                         0,
                         nullptr,
                         0,
                         nullptr,
                         1,
                         &to_color);

    VkClearValue clear_color{};
    clear_color.color = {{0.02f, 0.02f, 0.05f, 1.0f}};

    VkRenderingAttachmentInfo color_attach{};
    color_attach.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO_KHR;
    color_attach.pNext = nullptr;
    color_attach.imageView = g_state.swapchain_image_views[image_index];
    color_attach.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    color_attach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color_attach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color_attach.clearValue = clear_color;

    VkRenderingInfo rendering_info{};
    rendering_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO_KHR;
    rendering_info.pNext = nullptr;
    rendering_info.renderArea.offset = {0, 0};
    rendering_info.renderArea.extent = g_state.swapchain_extent;
    rendering_info.layerCount = 1;
    rendering_info.colorAttachmentCount = 1;
    rendering_info.pColorAttachments = &color_attach;

    if (!g_state.cmd_begin_rendering || !g_state.cmd_end_rendering) {
      rkg::log::warn("debug_ui render skipped: dynamic rendering entrypoints missing");
      return vkEndCommandBuffer(cmd) == VK_SUCCESS;
    }
#if RKG_ENABLE_IMGUI
    log_step("dynamic rendering: begin (renderer)");
#endif
    g_state.cmd_begin_rendering(cmd, &rendering_info);
    if (g_state.skip_imgui_frame) {
      log_step("imgui: skip render (viewport resized)");
      g_state.skip_imgui_frame = false;
    } else {
      if (rkg::debug_ui::is_initialized()) {
        log_step("imgui: render dynamic");
      }
      rkg::debug_ui::render(cmd);
    }
    log_step("dynamic rendering: end (renderer)");
    g_state.cmd_end_rendering(cmd);

    VkImageMemoryBarrier to_present{};
    to_present.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    to_present.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    to_present.dstAccessMask = 0;
    to_present.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    to_present.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    to_present.image = g_state.swapchain_images[image_index];
    to_present.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_present.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_present.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    to_present.subresourceRange.baseMipLevel = 0;
    to_present.subresourceRange.levelCount = 1;
    to_present.subresourceRange.baseArrayLayer = 0;
    to_present.subresourceRange.layerCount = 1;
    log_step("dynamic rendering: barrier to present");
    vkCmdPipelineBarrier(cmd,
                         VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                         VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
                         0,
                         0,
                         nullptr,
                         0,
                         nullptr,
                         1,
                         &to_present);

    if (image_index < g_state.swapchain_image_layouts.size()) {
      g_state.swapchain_image_layouts[image_index] = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    }
  }
#endif

  log_step("vkEndCommandBuffer");
  const VkResult end_result = vkEndCommandBuffer(cmd);
  if (end_result != VK_SUCCESS) {
    rkg::log::error(std::string("renderer:vulkan vkEndCommandBuffer failed: ") +
                    vk_result_name(end_result) + " (" + std::to_string(end_result) + ")");
    return false;
  }
  return true;
}

bool draw_frame() {
  static bool logged_state = false;
  static bool logged_defer = false;
  static bool logged_rebuild = false;

  if (g_state.device_lost) {
    rkg::log::error("renderer:vulkan device lost; skipping frame");
    return false;
  }

  if (g_state.swapchain == VK_NULL_HANDLE || g_state.swapchain_images.empty()) {
    g_state.swapchain_needs_rebuild = true;
  }

  if (g_state.swapchain_needs_rebuild) {
    int w = 0;
    int h = 0;
    get_window_drawable_size(w, h);
    if (w == 0 || h == 0) {
      if (!logged_defer) {
        rkg::log::warn("renderer:vulkan swapchain rebuild deferred (window size 0)");
        logged_defer = true;
      }
      rkg::commit_vulkan_viewport_request();
      return true;
    }
    logged_defer = false;
    if (!logged_rebuild) {
      rkg::log::warn("renderer:vulkan swapchain rebuild requested");
      logged_rebuild = true;
    }
    if (!recreate_swapchain()) {
      rkg::log::warn("renderer:vulkan swapchain rebuild failed; will retry");
      return true;
    }
    g_state.swapchain_needs_rebuild = false;
    logged_rebuild = false;
    return true;
  }

  if (g_state.swapchain_extent.width == 0 || g_state.swapchain_extent.height == 0) {
    if (!logged_state) {
      rkg::log::warn("renderer:vulkan swapchain extent is 0; skipping frame");
      logged_state = true;
    }
    rkg::commit_vulkan_viewport_request();
    return true;
  }

  log_step("draw_frame: wait fence");
  const VkResult fence_result = vkWaitForFences(g_state.device, 1, &g_state.in_flight_fence, VK_TRUE, UINT64_MAX);
  if (fence_result != VK_SUCCESS) {
    rkg::log::error(std::string("renderer:vulkan vkWaitForFences failed: ") +
                    vk_result_name(fence_result) + " (" + std::to_string(fence_result) + ")");
    if (fence_result == VK_ERROR_DEVICE_LOST) {
      g_state.device_lost = true;
      return false;
    }
    return true;
  }

  // Only touch (destroy/recreate) offscreen resources after the in-flight fence has completed.
  update_offscreen_target();

  if (!logged_state) {
    rkg::log::info(std::string("renderer:vulkan swapchain=") + std::to_string(reinterpret_cast<uintptr_t>(g_state.swapchain)));
    rkg::log::info("renderer:vulkan swapchain images=" + std::to_string(g_state.swapchain_images.size()) +
                   " views=" + std::to_string(g_state.swapchain_image_views.size()) +
                   " framebuffers=" + std::to_string(g_state.framebuffers.size()) +
                   " cmd_buffers=" + std::to_string(g_state.command_buffers.size()));
    rkg::log::info("renderer:vulkan extent=" + std::to_string(g_state.swapchain_extent.width) + "x" +
                   std::to_string(g_state.swapchain_extent.height));
    logged_state = true;
  }

  log_step("vkAcquireNextImageKHR");
  uint32_t image_index = 0;
  VkResult result = vkAcquireNextImageKHR(
      g_state.device, g_state.swapchain, UINT64_MAX, g_state.image_available, VK_NULL_HANDLE, &image_index);
  if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
    rkg::log::warn(std::string("renderer:vulkan swapchain acquire: ") + vk_result_name(result));
    g_state.swapchain_needs_rebuild = true;
    rkg::commit_vulkan_viewport_request();
    return true;
  }
  if (result == VK_ERROR_SURFACE_LOST_KHR) {
    rkg::log::error("renderer:vulkan surface lost during acquire");
    g_state.surface_lost = true;
    g_state.swapchain_needs_rebuild = true;
    rkg::commit_vulkan_viewport_request();
    return true;
  }
  if (result != VK_SUCCESS) {
    rkg::log::error(std::string("renderer:vulkan vkAcquireNextImageKHR failed: ") +
                    vk_result_name(result) + " (" + std::to_string(result) + ")");
    if (result == VK_ERROR_DEVICE_LOST) {
      g_state.device_lost = true;
      return false;
    }
    g_state.swapchain_needs_rebuild = true;
    rkg::commit_vulkan_viewport_request();
    return true;
  }
  if (image_index >= g_state.command_buffers.size() || image_index >= g_state.swapchain_image_views.size() ||
      image_index >= g_state.framebuffers.size()) {
    rkg::log::error("renderer:vulkan swapchain image index out of range");
    rkg::commit_vulkan_viewport_request();
    return false;
  }

  log_step("vkResetCommandBuffer");
  vkResetCommandBuffer(g_state.command_buffers[image_index], 0);
  if (!record_command_buffer(g_state.command_buffers[image_index], image_index)) {
    rkg::log::warn("renderer:vulkan record_command_buffer failed");
    g_state.swapchain_needs_rebuild = true;
    rkg::commit_vulkan_viewport_request();
    return true;
  }

  VkSemaphore wait_semaphores[] = {g_state.image_available};
  VkPipelineStageFlags wait_stages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
  VkSemaphore signal_semaphores[] = {g_state.render_finished};

  VkSubmitInfo submit_info{};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit_info.waitSemaphoreCount = 1;
  submit_info.pWaitSemaphores = wait_semaphores;
  submit_info.pWaitDstStageMask = wait_stages;
  submit_info.commandBufferCount = 1;
  submit_info.pCommandBuffers = &g_state.command_buffers[image_index];
  submit_info.signalSemaphoreCount = 1;
  submit_info.pSignalSemaphores = signal_semaphores;

  log_step("vkQueueSubmit");
  vkResetFences(g_state.device, 1, &g_state.in_flight_fence);
  const VkResult submit_result = vkQueueSubmit(g_state.graphics_queue, 1, &submit_info, g_state.in_flight_fence);
  if (submit_result != VK_SUCCESS) {
    rkg::log::error(std::string("renderer:vulkan vkQueueSubmit failed: ") +
                    vk_result_name(submit_result) + " (" + std::to_string(submit_result) + ")");
    if (submit_result == VK_ERROR_DEVICE_LOST) {
      g_state.device_lost = true;
      rkg::commit_vulkan_viewport_request();
      return false;
    }
    g_state.swapchain_needs_rebuild = true;
    rkg::commit_vulkan_viewport_request();
    return true;
  }

  VkPresentInfoKHR present_info{};
  present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
  present_info.waitSemaphoreCount = 1;
  present_info.pWaitSemaphores = signal_semaphores;
  present_info.swapchainCount = 1;
  present_info.pSwapchains = &g_state.swapchain;
  present_info.pImageIndices = &image_index;

  log_step("vkQueuePresentKHR");
  result = vkQueuePresentKHR(g_state.present_queue, &present_info);
  rkg::commit_vulkan_viewport_request();
  if (result == VK_ERROR_SURFACE_LOST_KHR) {
    rkg::log::error("renderer:vulkan surface lost during present");
    g_state.surface_lost = true;
    g_state.swapchain_needs_rebuild = true;
    return true;
  }
  if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
    rkg::log::warn(std::string("renderer:vulkan present: ") + vk_result_name(result));
    g_state.swapchain_needs_rebuild = true;
    return true;
  }
  if (result == VK_SUCCESS) {
    g_log_steps = false;
    return true;
  }
  rkg::log::error(std::string("renderer:vulkan vkQueuePresentKHR failed: ") + vk_result_name(result));
  return false;
}

bool vulkan_init(void* host) {
  g_state.ctx = static_cast<rkg::HostContext*>(host);
  if (!g_state.ctx || !g_state.ctx->platform) {
    rkg::log::error("renderer:vulkan missing platform");
    return false;
  }

  g_state.window = static_cast<SDL_Window*>(g_state.ctx->platform->native_window());
  if (!g_state.window) {
    rkg::log::error("renderer:vulkan missing SDL window");
    return false;
  }

  if (!create_instance()) return false;
  if (!create_surface()) return false;
  if (!pick_physical_device()) return false;
  if (!find_queue_families()) return false;
  if (!create_device()) return false;
  if (!create_swapchain()) return false;
  if (!create_render_pass()) return false;
  if (!create_framebuffers()) return false;
  if (!create_command_pool()) return false;
  if (!create_command_buffers()) return false;
  if (rkg::get_vulkan_viewport_textured_demo_enabled()) {
    if (!load_textured_asset()) {
      rkg::log::warn("renderer:vulkan textured demo asset not ready");
    }
  }
  if (!create_sync_objects()) return false;

  rkg::VulkanHooks hooks;
  hooks.instance = g_state.instance;
  hooks.physical_device = g_state.physical_device;
  hooks.device = g_state.device;
  hooks.queue = g_state.graphics_queue;
  hooks.render_pass = g_state.render_pass;
  hooks.window = g_state.window;
  hooks.queue_family = g_state.graphics_queue_family;
  hooks.image_count = static_cast<uint32_t>(g_state.swapchain_images.size());
  hooks.swapchain_format = static_cast<uint32_t>(g_state.swapchain_format);
  rkg::register_vulkan_hooks(&hooks);

  rkg::log::info("renderer:vulkan init");
  rkg::log::info(std::string("renderer:vulkan dynamic rendering: ") +
                 (g_state.dynamic_rendering_enabled ? "enabled" : "disabled"));
  return true;
}

void vulkan_shutdown() {
  rkg::register_vulkan_hooks(nullptr);
  destroy_vulkan();
  rkg::log::info("renderer:vulkan shutdown");
}

void vulkan_on_window_resized(int, int) {
  g_state.swapchain_needs_rebuild = true;
}

void vulkan_update(float dt) {
  static bool logged = false;
  if (!logged) {
    g_log_steps = true;
    g_log_step_index = 0;
    rkg::log::info("renderer:vulkan update begin");
  }
  if (dt > 0.0f && dt < 1.0f) {
    g_state.frame_dt = dt;
  }
  if (!draw_frame()) {
    rkg::log::warn("renderer:vulkan draw_frame failed");
  }
  if (!logged) {
    rkg::log::info("renderer:vulkan update end");
    logged = true;
  }
}

rkg::RkgPluginApi g_api{
    rkg::kRkgPluginApiVersion,
    "renderer_vulkan",
    rkg::PluginType::Renderer,
    static_cast<uint32_t>(rkg::RendererCaps::DrawPrimitive) |
        static_cast<uint32_t>(rkg::RendererCaps::DebugUiSupported),
    &vulkan_init,
    &vulkan_shutdown,
    &vulkan_update,
    &vulkan_on_window_resized};
} // namespace

extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_renderer_vulkan(uint32_t host_api_version) {
  if (host_api_version != rkg::kRkgPluginApiVersion) {
    return nullptr;
  }
  return &g_api;
}

#if defined(RKG_BUILD_DYNAMIC_PLUGIN)
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api(uint32_t host_api_version) {
  return rkg_plugin_get_api_renderer_vulkan(host_api_version);
}
#endif
