#pragma once

#include <filesystem>
#include <string>
#include <vulkan/vulkan.h>

namespace rkg::debug_ui {

using DrawCallback = void (*)(void* user_data);

bool init_vulkan();
void shutdown();
void new_frame(float dt_seconds);
void render(VkCommandBuffer cmd);
void set_visible(bool visible);
bool is_visible();
void process_event(const void* event);
void set_host_context(void* host_context);
void set_root_path(const std::filesystem::path& root);
void set_renderer_name(const std::string& name);
void set_hot_reload_status(const std::string& time, const std::string& error);
bool consume_force_reload_request();
void set_viewport_size(int width, int height);
bool viewport_supported();
void* viewport_texture();
void viewport_size(int* width, int* height);
const char* viewport_error();
void set_draw_callback(DrawCallback callback, void* user_data);
void set_show_builtin(bool show);
void set_docking_enabled(bool enabled);

} // namespace rkg::debug_ui
