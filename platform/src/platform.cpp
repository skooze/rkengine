#include "rkg_platform/platform.h"

namespace rkg::platform::detail {
bool platform_init(Platform* self, const WindowDesc& desc);
void platform_shutdown(Platform* self);
void platform_poll_events(Platform* self);
bool platform_should_quit(const Platform* self);
float platform_delta_seconds(Platform* self);
void* platform_native_window(const Platform* self);
} // namespace rkg::platform::detail

namespace rkg::platform {

bool Platform::init(const WindowDesc& desc) {
  return detail::platform_init(this, desc);
}

void Platform::shutdown() {
  detail::platform_shutdown(this);
}

void Platform::poll_events() {
  detail::platform_poll_events(this);
}

bool Platform::should_quit() const {
  return detail::platform_should_quit(this);
}

float Platform::delta_seconds() {
  return detail::platform_delta_seconds(this);
}

void* Platform::native_window() const {
  return detail::platform_native_window(this);
}

bool Platform::is_key_down(KeyCode code) const {
  return keys_down_.find(code) != keys_down_.end();
}

void Platform::request_quit() {
  quit_ = true;
}

void Platform::set_event_callback(EventCallback callback, void* user_data) {
  event_callback_ = callback;
  event_user_data_ = user_data;
}

} // namespace rkg::platform
