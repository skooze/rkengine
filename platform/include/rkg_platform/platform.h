#pragma once

#include <unordered_set>

namespace rkg::platform {

enum class KeyCode {
  Unknown = 0,
  W,
  A,
  S,
  D,
  Escape,
  F1,
  F5
};

struct WindowDesc {
  int width = 1280;
  int height = 720;
  const char* title = "rkg";
};

class Platform;

namespace detail {
bool platform_init(Platform* self, const WindowDesc& desc);
void platform_shutdown(Platform* self);
void platform_poll_events(Platform* self);
bool platform_should_quit(const Platform* self);
float platform_delta_seconds(Platform* self);
void* platform_native_window(const Platform* self);
} // namespace detail

class Platform {
 public:
  using EventCallback = void (*)(const void* event, void* user_data);

  bool init(const WindowDesc& desc);
  void shutdown();
  void poll_events();
  bool should_quit() const;
  float delta_seconds();
  void* native_window() const;
  bool is_key_down(KeyCode code) const;
  void request_quit();
  void set_event_callback(EventCallback callback, void* user_data);

 private:
  friend bool detail::platform_init(Platform* self, const WindowDesc& desc);
  friend void detail::platform_shutdown(Platform* self);
  friend void detail::platform_poll_events(Platform* self);
  friend bool detail::platform_should_quit(const Platform* self);
  friend float detail::platform_delta_seconds(Platform* self);
  friend void* detail::platform_native_window(const Platform* self);

  bool quit_ = false;
  unsigned long long last_ticks_ = 0;
  void* window_ = nullptr;
  std::unordered_set<KeyCode> keys_down_;
  EventCallback event_callback_ = nullptr;
  void* event_user_data_ = nullptr;
};

} // namespace rkg::platform
