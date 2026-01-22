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
};

void register_vulkan_hooks(const VulkanHooks* hooks);
const VulkanHooks* get_vulkan_hooks();

} // namespace rkg
