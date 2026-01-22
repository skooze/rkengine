#include "rkg/renderer_hooks.h"

namespace rkg {

namespace {
VulkanHooks g_hooks{};
bool g_has_hooks = false;
} // namespace

void register_vulkan_hooks(const VulkanHooks* hooks) {
  if (!hooks) {
    g_has_hooks = false;
    g_hooks = VulkanHooks{};
    return;
  }
  g_hooks = *hooks;
  g_has_hooks = true;
}

const VulkanHooks* get_vulkan_hooks() {
  return g_has_hooks ? &g_hooks : nullptr;
}

} // namespace rkg
