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
void commit_vulkan_viewport_request();

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

struct VulkanViewportCamera {
  float view_proj[16]{};
  uint64_t version = 0;
};

void set_vulkan_viewport_camera(const float* view_proj);
const VulkanViewportCamera* get_vulkan_viewport_camera();

struct VulkanViewportLineList {
  static constexpr uint32_t kMaxLines = 512;
  float positions[kMaxLines * 6]{};
  float colors[kMaxLines * 4]{};
  uint32_t line_count = 0;
  uint64_t version = 0;
};

void set_vulkan_viewport_line_list(const float* positions,
                                   const float* colors,
                                   uint32_t line_count);
const VulkanViewportLineList* get_vulkan_viewport_line_list();

struct VulkanViewportTexturedDemo {
  float mvp[16]{};
  bool has_mvp = false;
  uint64_t version = 0;
};

void set_vulkan_viewport_textured_demo_mvp(const float* mvp);
const VulkanViewportTexturedDemo* get_vulkan_viewport_textured_demo();

void set_vulkan_viewport_textured_demo_enabled(bool enabled);
bool get_vulkan_viewport_textured_demo_enabled();

} // namespace rkg
