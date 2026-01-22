#include "rkg_platform/platform.h"

#include "rkg/log.h"

namespace rkg::platform::detail {

bool platform_init(Platform* self, const WindowDesc& desc) {
  (void)desc;
  self->quit_ = true;
  rkg::log::warn("platform init stub: Windows support not implemented yet.");
  return false;
}

void platform_shutdown(Platform* self) {
  (void)self;
}

void platform_poll_events(Platform* self) {
  (void)self;
}

bool platform_should_quit(const Platform* self) {
  return self->quit_;
}

float platform_delta_seconds(Platform* self) {
  (void)self;
  return 0.0f;
}

void* platform_native_window(const Platform* self) {
  (void)self;
  return nullptr;
}

} // namespace rkg::platform::detail
