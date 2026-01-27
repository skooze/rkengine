#include "rkg/renderer_hooks.h"

#include <cstddef>

namespace rkg {

namespace {
VulkanHooks g_hooks{};
bool g_has_hooks = false;
VulkanViewportHooks g_viewport{};
bool g_has_viewport = false;
uint32_t g_viewport_request_width = 0;
uint32_t g_viewport_request_height = 0;
uint32_t g_viewport_active_width = 0;
uint32_t g_viewport_active_height = 0;
bool g_viewport_request_dirty = false;
VulkanViewportDrawList g_viewport_draw_list{};
VulkanViewportCamera g_viewport_camera{};
VulkanViewportLineList g_viewport_line_list{};
VulkanViewportTexturedDemo g_viewport_textured_demo{};
bool g_viewport_textured_demo_enabled = true;
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

void register_vulkan_viewport(const VulkanViewportHooks* hooks) {
  if (!hooks) {
    g_has_viewport = false;
    g_viewport = VulkanViewportHooks{};
    return;
  }
  g_viewport = *hooks;
  g_has_viewport = true;
}

const VulkanViewportHooks* get_vulkan_viewport() {
  return g_has_viewport ? &g_viewport : nullptr;
}

void set_vulkan_viewport_request(uint32_t width, uint32_t height) {
  g_viewport_request_width = width;
  g_viewport_request_height = height;
  g_viewport_request_dirty = true;
}

void get_vulkan_viewport_request(uint32_t& width, uint32_t& height) {
  width = g_viewport_active_width;
  height = g_viewport_active_height;
}

void commit_vulkan_viewport_request() {
  if (!g_viewport_request_dirty) {
    return;
  }
  g_viewport_active_width = g_viewport_request_width;
  g_viewport_active_height = g_viewport_request_height;
  g_viewport_request_dirty = false;
}

void set_vulkan_viewport_draw_list(const float* mvp,
                                   const uint32_t* mesh_id,
                                   const float* color,
                                   uint32_t instance_count) {
  if (!mvp || instance_count == 0) {
    g_viewport_draw_list.instance_count = 0;
    g_viewport_draw_list.version += 1;
    return;
  }
  if (instance_count > VulkanViewportDrawList::kMaxInstances) {
    instance_count = VulkanViewportDrawList::kMaxInstances;
  }
  const size_t count = static_cast<size_t>(instance_count) * 16;
  for (size_t i = 0; i < count; ++i) {
    g_viewport_draw_list.mvp[i] = mvp[i];
  }
  for (size_t i = 0; i < instance_count; ++i) {
    g_viewport_draw_list.mesh_id[i] = mesh_id ? mesh_id[i] : 0u;
  }
  const size_t color_count = static_cast<size_t>(instance_count) * 4;
  if (color) {
    for (size_t i = 0; i < color_count; ++i) {
      g_viewport_draw_list.color[i] = color[i];
    }
  } else {
    for (size_t i = 0; i < color_count; ++i) {
      g_viewport_draw_list.color[i] = 1.0f;
    }
  }
  g_viewport_draw_list.instance_count = instance_count;
  g_viewport_draw_list.version += 1;
}

const VulkanViewportDrawList* get_vulkan_viewport_draw_list() {
  return &g_viewport_draw_list;
}

void set_vulkan_viewport_camera(const float* view_proj) {
  if (!view_proj) {
    g_viewport_camera.version += 1;
    return;
  }
  for (size_t i = 0; i < 16; ++i) {
    g_viewport_camera.view_proj[i] = view_proj[i];
  }
  g_viewport_camera.version += 1;
}

const VulkanViewportCamera* get_vulkan_viewport_camera() {
  return &g_viewport_camera;
}

void set_vulkan_viewport_line_list(const float* positions,
                                   const float* colors,
                                   uint32_t line_count) {
  if (!positions || line_count == 0) {
    g_viewport_line_list.line_count = 0;
    g_viewport_line_list.version += 1;
    return;
  }
  if (line_count > VulkanViewportLineList::kMaxLines) {
    line_count = VulkanViewportLineList::kMaxLines;
  }
  const size_t pos_count = static_cast<size_t>(line_count) * 6;
  for (size_t i = 0; i < pos_count; ++i) {
    g_viewport_line_list.positions[i] = positions[i];
  }
  const size_t color_count = static_cast<size_t>(line_count) * 4;
  if (colors) {
    for (size_t i = 0; i < color_count; ++i) {
      g_viewport_line_list.colors[i] = colors[i];
    }
  } else {
    for (size_t i = 0; i < color_count; ++i) {
      g_viewport_line_list.colors[i] = 1.0f;
    }
  }
  g_viewport_line_list.line_count = line_count;
  g_viewport_line_list.version += 1;
}

const VulkanViewportLineList* get_vulkan_viewport_line_list() {
  return &g_viewport_line_list;
}

void set_vulkan_viewport_textured_demo_mvp(const float* mvp) {
  if (!mvp) {
    g_viewport_textured_demo.has_mvp = false;
    g_viewport_textured_demo.version += 1;
    return;
  }
  for (size_t i = 0; i < 16; ++i) {
    g_viewport_textured_demo.mvp[i] = mvp[i];
  }
  g_viewport_textured_demo.has_mvp = true;
  g_viewport_textured_demo.version += 1;
}

const VulkanViewportTexturedDemo* get_vulkan_viewport_textured_demo() {
  return &g_viewport_textured_demo;
}

void set_vulkan_viewport_textured_demo_enabled(bool enabled) {
  g_viewport_textured_demo_enabled = enabled;
}

bool get_vulkan_viewport_textured_demo_enabled() {
  return g_viewport_textured_demo_enabled;
}

} // namespace rkg
