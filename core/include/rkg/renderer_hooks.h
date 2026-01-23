#pragma once

#include <cstdint>

namespace rkg {

struct VulkanHooks {
  void* instance = nullptr;
  void* physical_device = nullptr;
  void* device = nullptr;
  void* queue = nullptr;
  void* render_pass = nullptr;
  void* window = nullptr;
  uint32_t queue_family = 0;
  uint32_t image_count = 0;
  uint32_t swapchain_format = 0;
};

void register_vulkan_hooks(const VulkanHooks* hooks);
const VulkanHooks* get_vulkan_hooks();

struct VulkanViewportHooks {
  void* image_view = nullptr;
  void* sampler = nullptr;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t layout = 0;
  uint64_t version = 0;
};

void register_vulkan_viewport(const VulkanViewportHooks* hooks);
const VulkanViewportHooks* get_vulkan_viewport();
void set_vulkan_viewport_request(uint32_t width, uint32_t height);
void get_vulkan_viewport_request(uint32_t& width, uint32_t& height);

struct VulkanViewportDrawList {
  static constexpr uint32_t kMaxInstances = 64;
  float mvp[kMaxInstances * 16]{};
  float color[kMaxInstances * 4]{};
  uint32_t mesh_id[kMaxInstances]{};
  uint32_t instance_count = 0;
  uint64_t version = 0;
};

void set_vulkan_viewport_draw_list(const float* mvp,
                                   const uint32_t* mesh_id,
                                   const float* color,
                                   uint32_t instance_count);
const VulkanViewportDrawList* get_vulkan_viewport_draw_list();

} // namespace rkg
