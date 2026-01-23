#include "rkg/host_context.h"
#include "rkg/log.h"
#include "rkg/plugin_api.h"
#include "rkg/renderer_hooks.h"
#include "rkg_platform/platform.h"

#if RKG_ENABLE_IMGUI
#include "rkg_debug_ui/imgui_api.h"
#endif
#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>
#include <vulkan/vulkan.h>

#include <algorithm>
#include <cstring>
#include <vector>

namespace {

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
  VkRenderPass render_pass = VK_NULL_HANDLE;
  std::vector<VkFramebuffer> framebuffers;
  VkCommandPool command_pool = VK_NULL_HANDLE;
  std::vector<VkCommandBuffer> command_buffers;
  VkSemaphore image_available = VK_NULL_HANDLE;
  VkSemaphore render_finished = VK_NULL_HANDLE;
  VkFence in_flight_fence = VK_NULL_HANDLE;

  VkImage offscreen_image = VK_NULL_HANDLE;
  VkDeviceMemory offscreen_memory = VK_NULL_HANDLE;
  VkImageView offscreen_view = VK_NULL_HANDLE;
  VkSampler offscreen_sampler = VK_NULL_HANDLE;
  VkRenderPass offscreen_render_pass = VK_NULL_HANDLE;
  VkFramebuffer offscreen_framebuffer = VK_NULL_HANDLE;
  VkExtent2D offscreen_extent{};
  uint64_t offscreen_version = 0;

  VkImage offscreen_depth_image = VK_NULL_HANDLE;
  VkDeviceMemory offscreen_depth_memory = VK_NULL_HANDLE;
  VkImageView offscreen_depth_view = VK_NULL_HANDLE;
  VkFormat offscreen_depth_format = VK_FORMAT_D32_SFLOAT;

  VkPipeline viewport_pipeline = VK_NULL_HANDLE;
  VkPipelineLayout viewport_pipeline_layout = VK_NULL_HANDLE;
  VkBuffer viewport_cube_buffer = VK_NULL_HANDLE;
  VkDeviceMemory viewport_cube_memory = VK_NULL_HANDLE;
  VkBuffer viewport_quad_buffer = VK_NULL_HANDLE;
  VkDeviceMemory viewport_quad_memory = VK_NULL_HANDLE;
  uint32_t viewport_cube_vertex_count = 0;
  uint32_t viewport_quad_vertex_count = 0;
};

VulkanState g_state{};

bool create_viewport_pipeline();
bool create_viewport_vertex_buffer();

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

  const char* extensions[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
  VkDeviceCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  create_info.queueCreateInfoCount = static_cast<uint32_t>(queue_infos.size());
  create_info.pQueueCreateInfos = queue_infos.data();
  create_info.enabledExtensionCount = 1;
  create_info.ppEnabledExtensionNames = extensions;

  if (vkCreateDevice(g_state.physical_device, &create_info, nullptr, &g_state.device) != VK_SUCCESS) {
    rkg::log::error("vkCreateDevice failed");
    return false;
  }

  vkGetDeviceQueue(g_state.device, g_state.graphics_queue_family, 0, &g_state.graphics_queue);
  vkGetDeviceQueue(g_state.device, g_state.present_queue_family, 0, &g_state.present_queue);
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

void destroy_offscreen() {
  if (g_state.viewport_pipeline != VK_NULL_HANDLE) {
    vkDestroyPipeline(g_state.device, g_state.viewport_pipeline, nullptr);
    g_state.viewport_pipeline = VK_NULL_HANDLE;
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
  g_state.viewport_cube_vertex_count = 0;
  g_state.viewport_quad_vertex_count = 0;

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
  attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  attachments[0].finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

  attachments[1].format = g_state.offscreen_depth_format;
  attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
  attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
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
  if (!create_viewport_vertex_buffer()) {
    rkg::log::error("viewport vertex buffer setup failed");
    return false;
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
    }
    return;
  }
  if (g_state.offscreen_extent.width != req_w || g_state.offscreen_extent.height != req_h) {
    create_offscreen(req_w, req_h);
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
  raster.cullMode = VK_CULL_MODE_BACK_BIT;
  raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  raster.depthBiasEnable = VK_FALSE;

  VkPipelineMultisampleStateCreateInfo multisample{};
  multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

  VkPipelineDepthStencilStateCreateInfo depth{};
  depth.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
  depth.depthTestEnable = VK_TRUE;
  depth.depthWriteEnable = VK_TRUE;
  depth.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
  depth.depthBoundsTestEnable = VK_FALSE;
  depth.stencilTestEnable = VK_FALSE;

  VkPipelineColorBlendAttachmentState color_blend_attach{};
  color_blend_attach.colorWriteMask =
      VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  color_blend_attach.blendEnable = VK_FALSE;

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

bool create_viewport_vertex_buffer() {
  if (g_state.viewport_cube_buffer != VK_NULL_HANDLE && g_state.viewport_quad_buffer != VK_NULL_HANDLE) return true;

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
  SDL_GetWindowSize(g_state.window, &w, &h);
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
  color_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
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

void destroy_swapchain() {
  for (auto fb : g_state.framebuffers) {
    vkDestroyFramebuffer(g_state.device, fb, nullptr);
  }
  g_state.framebuffers.clear();

  if (g_state.render_pass != VK_NULL_HANDLE) {
    vkDestroyRenderPass(g_state.device, g_state.render_pass, nullptr);
    g_state.render_pass = VK_NULL_HANDLE;
  }

  for (auto view : g_state.swapchain_image_views) {
    vkDestroyImageView(g_state.device, view, nullptr);
  }
  g_state.swapchain_image_views.clear();

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
  VkCommandBufferBeginInfo begin_info{};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  if (vkBeginCommandBuffer(cmd, &begin_info) != VK_SUCCESS) {
    return false;
  }

  if (g_state.offscreen_render_pass != VK_NULL_HANDLE && g_state.offscreen_framebuffer != VK_NULL_HANDLE) {
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
    vkCmdEndRenderPass(cmd);
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

  vkCmdClearAttachments(cmd, 1, &clear_attachment, 1, &rect);

#if RKG_ENABLE_IMGUI
  rkg::debug_ui::render(cmd);
#endif

  vkCmdEndRenderPass(cmd);

  return vkEndCommandBuffer(cmd) == VK_SUCCESS;
}

bool draw_frame() {
  vkWaitForFences(g_state.device, 1, &g_state.in_flight_fence, VK_TRUE, UINT64_MAX);
  vkResetFences(g_state.device, 1, &g_state.in_flight_fence);

  update_offscreen_target();

  uint32_t image_index = 0;
  VkResult result = vkAcquireNextImageKHR(
      g_state.device, g_state.swapchain, UINT64_MAX, g_state.image_available, VK_NULL_HANDLE, &image_index);
  if (result != VK_SUCCESS) {
    return false;
  }

  vkResetCommandBuffer(g_state.command_buffers[image_index], 0);
  if (!record_command_buffer(g_state.command_buffers[image_index], image_index)) {
    return false;
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

  if (vkQueueSubmit(g_state.graphics_queue, 1, &submit_info, g_state.in_flight_fence) != VK_SUCCESS) {
    return false;
  }

  VkPresentInfoKHR present_info{};
  present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
  present_info.waitSemaphoreCount = 1;
  present_info.pWaitSemaphores = signal_semaphores;
  present_info.swapchainCount = 1;
  present_info.pSwapchains = &g_state.swapchain;
  present_info.pImageIndices = &image_index;

  result = vkQueuePresentKHR(g_state.present_queue, &present_info);
  return result == VK_SUCCESS;
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
  rkg::register_vulkan_hooks(&hooks);

  rkg::log::info("renderer:vulkan init");
  return true;
}

void vulkan_shutdown() {
  rkg::register_vulkan_hooks(nullptr);
  destroy_vulkan();
  rkg::log::info("renderer:vulkan shutdown");
}

void vulkan_update(float) {
  if (!draw_frame()) {
    rkg::log::warn("renderer:vulkan draw_frame failed");
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
    nullptr};
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
